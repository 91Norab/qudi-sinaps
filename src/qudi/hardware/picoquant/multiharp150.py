# -*- coding: utf-8 -*-
"""
This file contains the Qudi hardware module for the MultiHarp150.

Copyright (c) 2021, the Qudi developers. 
See the AUTHORS.md file at the top-level directory of this
distribution and on <https://github.com/Ulm-IQO/qudi-iqo-modules/>

This file is part of Qudi.

Qudi is free software: you can redistribute it and/or modify it under the terms of
the GNU Lesser General Public License as published by the Free Software Foundation,
either version 3 of the License, or (at your option) any later version.

Qudi is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with Qudi.
If not, see <https://www.gnu.org/licenses/>.
"""

import ctypes
import numpy as np
import time
import os
from fractions import Fraction
from qtpy import QtCore

from qudi.core.configoption import ConfigOption
from qudi.util.overload import OverloadedAttribute

from qudi.interface.fast_counter_interface import FastCounterInterface
from qudi.interface.slow_counter_interface import SlowCounterInterface, \
    SlowCounterConstraints, CountingMode


class Multiharp150(FastCounterInterface, SlowCounterInterface):
    """
    Hardware class to control the MultiHarp 150 sold by PicoQuant.
    This class is written according to the Programming Library Version 3.1

    Example config for copy-paste:

    counter_multiharp150:
        module.Class: 'picoquant.multiharp150.MultiHarp150'
        options:
            deviceID: 0 # a device index from 0 to 7.
            mode: 0 # 0: histogram mode, 2: T2 mode, 3: T3 mode
    """

    _deviceID = ConfigOption('deviceID', 0, missing='warn')  # a device index from 0 to 7.
    _mode = ConfigOption('mode', 0, missing='warn')
    _ref_source = ConfigOption('ref_source', 0)
    _dll_name = ConfigOption('dll', 'mhlib64')

    _clock_frequency = ConfigOption('clock_frequency', 10)

    _count_channel = ConfigOption('count_channel', 0)
    _input_channel_offset = ConfigOption('input_channel_offset', 0)
    _sync_div = ConfigOption('sync_div', 1)
    _sync_channel_offset = ConfigOption('sync_channel_offset', 0)
    _PS_TO_S = 1e-12

    sigStart = QtCore.Signal()

    """
    # ========================================
    #              Basic functions
    # ========================================
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Load the multiharp library file mhlib64.dll from the folder <Windows>/System32/:
        self._dll = ctypes.cdll.LoadLibrary(self._dll_name)

        # Init check params: (the library can communicate with 8 devices)
        self.connected_to_device = False
        self.meas_run = False
        self._slow_or_fast = 'slow'

        # Init hardware:
        self.error_code = self._create_error_code()
        self._set_constants()
        self._len_code = self._max_len_code  # histogram length code, set to maximum value
        self._base_resolution_s = None

        #  Init for fast counter
        self.result = []
        self._data_trace = None
        self._fast_bin_width_s = None
        self._fast_record_length_s = None
        self._fast_number_of_gates = None

    def on_activate(self):
        """ Activate and establish the connection to the MultiHarp and initialize. """
        self._open_connection()
        self._initialize(self._mode, self._ref_source)

        # Init sync channel params (divider and offset)
        self._set_sync_div(self._sync_div)
        self._set_sync_channel_offset(self._sync_channel_offset)

        # Init counting channel params (enable and offset)
        self._set_input_channel_enable(self._count_channel, 1)
        self._set_input_channel_offset(self._count_channel, self._input_channel_offset)

        # Init multiharp150 config:
        # Set binning on activation to default value: 23 (max bin-steps - 1).
        # It corresponds to the worst resolution (41u), adapted for slow counting and scans (no timing goal).
        # We change this value for fast counting measurements in the configure for fast counting.
        # If back to slow counting, we set 23 (max bin-steps - 1) again.
        self._set_binning(23)
        self._set_histo_len(self._len_code)
        self._base_resolution_s = self._PS_TO_S * self._get_base_resolution()[0]

        self.exposure_time = 0.1
        self.set_up_counter()
        self.set_up_clock()

        # the signal has one argument of type object, which should allow anything to pass through:
        self.sigStart.connect(self.start_measure)

    def on_deactivate(self):
        """ Deactivates and disconnects the device. """
        self._close_connection()

    """
    # ========================================
    # Functions for the FastCounter Interface
    # ========================================
    """
    # FIXME: The interface connection to the fast counter must be established!

    get_constraints = OverloadedAttribute()
    @get_constraints.overload('FastCounterInterface')
    def get_constraints(self):
        """ Retrieve the hardware constrains from the Fast counting device.

        @return dict: dict with keys being the constraint names as string
                      and items are the definition for the constraints.

        The keys of the returned dictionary are the str name for the constraints (which are set in this method).

                    NO OTHER KEYS SHOULD BE INVENTED!

        If you are not sure about the meaning, look in other hardware files to get an impression.
        If still additional constraints are needed, then they have to be added to all files containing this interface.

        The items of the keys are again dictionaries which have the generic dictionary form:
            {'min': <value>,
             'max': <value>,
             'step': <value>,
             'unit': '<value>'}

        Only the key 'hardware_binwidth_list' differs, since they contain the list of possible binwidths.

        If the constraints cannot be set in the fast counting hardware
        then write just zero to each key of the generic dicts.
        Note that there is a difference between float input (0.0) and integer input (0),
        because some logic modules might rely on that distinction.

        ALL THE PRESENT KEYS OF THE CONSTRAINTS DICT MUST BE ASSIGNED!
        """
        constraints = dict()

        # The unit of those entries are seconds per bin. In order to get the current bin width in seconds,
        # use the get_binwidth method.
        bin_list = []
        for i in range(self._max_bin_steps):
            bin_list.append(5 * self._PS_TO_S * 2 ** i)

        constraints['hardware_binwidth_list'] = bin_list
        return constraints

    # configure = OverloadedAttribute()
    # @configure.overload('FastCounterInterface')
    def configure(self, fast_bin_width_s, fast_record_length_s, number_of_gates=0):
        """ Configuration of the fast counter.

        @params int bin_width_ns: Length of a single time bin in the time trace histogram in seconds.
        @params int record_length_s: Total length of the time trace/each single gate in seconds.
        @params int number_of_gates: Number of gates in the pulse sequence. Ignore for non-gated counter.
        @return int tuple(3): (bin width (sec), record length (sec), number of gates).
        """
        self._fast_bin_width_s = fast_bin_width_s
        self._fast_record_length_s = self._max_histogram_len * self._fast_bin_width_s
        self._fast_number_of_gates = number_of_gates
        self._data_trace = np.zeros(self._max_histogram_len)

        if not self.meas_run:
            self._slow_or_fast = 'fast'
            binning_code = int(np.log2(fast_bin_width_s / self._base_resolution_s))
            self._set_binning(binning_code)

        return self._fast_bin_width_s, self._fast_record_length_s, self._fast_number_of_gates

    def get_status(self):
        """ Receives the current status of the Fast Counter and outputs it as return value.
        0: not configured
        1: idle
        2: running
        3: paused
        -1: error state
        """
        if not self.connected_to_device:
            return -1
        if self.meas_run:
            return 2
        if self._fast_bin_width_s is None:
            return 0
        return 1

    def start_measure(self):
        """ Starts the fast counter.

        @return int: error code (0:OK, -1:error)
        """
        if self.meas_run and self._slow_or_fast == 'slow':
            return -1
        elif not self.meas_run and self._slow_or_fast == 'slow':
            self.configure(self._fast_bin_width_s, self._fast_record_length_s, self._fast_number_of_gates)
            self.meas_run = True
            self._clear_hist_memory()
            self._start(self.ACQTMAX)
            return 0
        else:
            self.meas_run = True
            self._clear_hist_memory()
            self._start(self.ACQTMAX)
            return 0

    def stop_measure(self):
        """ By setting the Flag, the measurement should stop. """
        if self._slow_or_fast == 'slow':
            return
        else:
            self._stop_meas()
            self.meas_run = False

    def pause_measure(self):
        """ Pauses the current measurement if the fast counter is in running state. """
        self.stop_measure()
        self.meas_run = False

    def continue_measure(self):
        """ Continues the current measurement if the fast counter is in pause state. """
        if self._slow_or_fast != 'fast':
            self.configure(self._fast_bin_width_s, self._fast_record_length_s, self._fast_number_of_gates)
        self.meas_run = True
        self._start(self.ACQTMAX)

    def is_gated(self):
        """ Boolean return value indicates if the fast counter is a gated counter (TRUE) or not (FALSE). """
        return False

    def get_binwidth(self):
        """ Returns the width of a single time bin in the time trace in seconds. """
        return self._fast_bin_width_s

    def get_data_trace(self, channel=0):
        """ Polls the current time_trace data from the fast counter and returns it as a numpy array (dtype = int64).
        The binning specified by calling configure() must be taken care of in this hardware class.
        A possible overflow of the histogram bins must be caught here and taken care of:
        - If the counter is NOT gated it will return a 1D-numpy-array with return array[time_bin_index].
        - If the counter is gated it will return a 2D-numpy-array with return array[gate_index, time_bin_index].
        """
        if self._slow_or_fast == 'fast':
            return self._get_histogram(self._count_channel)
        else:
            return

    """
    # ========================================
    #    Functions for SlowCounterInterface
    # ========================================
    """

    @get_constraints.overload('SlowCounterInterface')
    def get_constraints(self):
        """ Retrieve the hardware constrains from the counter device.

        @return (SlowCounterConstraints): object with constraints for the counter

        The constraints are defined as a SlowCounterConstraints object, defined at  the end of this file
        """
        constraints = SlowCounterConstraints()
        constraints.max_detectors = 1
        constraints.min_count_frequency = 1e-3
        constraints.max_count_frequency = 10e9
        constraints.counting_mode = [CountingMode.CONTINUOUS]
        return constraints

    def set_up_clock(self, clock_frequency=None, clock_channel=None):
        """ Set the frequency of the counter by configuring the hardware clock

        @param (float) clock_frequency: if defined, this sets the frequency of the clock
        @param (string) clock_channel: if defined, this is the physical channel of the clock
        @return int: error code (0:OK, -1:error)

        TODO: Should the logic know about the different clock channels ?
        """

        if clock_frequency is not None:
            self._clock_frequency = float(clock_frequency)
            self.exposure_time = 1 / self._clock_frequency
        else:
            self.exposure_time = 0.1

        return 0

    def set_up_counter(self, counter_channels=None, sources=None, clock_channel=None, counter_buffer=None):
        """ Configures the actual counter with a given clock.

        @param list(str) counter_channels: optional, physical channel of the counter
        @param list(str) sources: optional, physical channel where the photons are to count from
        @param str clock_channel: optional, specifies the clock channel for the counter
        @param int counter_buffer: optional, a buffer of specified integer length,
        where in each bin the count numbers are saved.

        @return int: error code (0:OK, -1:error)

        There need to be exactly the same numbers of sources and counter channels, and they need to be given
        in the same order. All counter channels share the same clock.
        """
        self._slow_or_fast = 'slow'
        self._set_binning(self._max_bin_steps - 1)
        # self._count_channel = counter_channel

        # Setting sync channel
        self._set_sync_edge_trg(-700, 0)

        # Setting counting channel (0, 1, 2 or 3 for multiharp150).
        self._set_input_dead_time(self._count_channel, 0, 800)
        self._set_input_edge_trg(self._count_channel, -300, 0)

        return 0

    def get_counter(self, samples=None):
        """ Returns the current counts per second of the counter.

        @param int samples: if defined, number of samples to read in one go

        @return numpy.array((n, uint32)): the measured quantity of each channel
        """
        if self.meas_run == True and self._slow_or_fast == 'fast':
            return np.zeros((1, 1))-1

        elif self.meas_run == False and self._slow_or_fast == 'fast':
            # No acquisition running but coming from a fast configuration
            self.set_up_counter(counter_channels=self._count_channel)
            self._slow_or_fast = 'slow'
            self.meas_run = True
            return self._get_counter_hist()

        elif self.meas_run == False and self._slow_or_fast == 'slow':
            self.meas_run = True
            return self._get_counter_hist()

        else:
            return self._get_counter_hist()

    def get_counter_channels(self):
        """ Returns the list of counter channel names.

        @return list(str): channel names

        Most methods calling this might just care about the number of channels, though.
        """
        return [str(self._count_channel)]

    def close_counter(self):
        """ Closes the counter and cleans up afterward.

        @return int: error code (0:OK, -1:error)
        """
        self._stop_meas()
        self.meas_run = False
        return 0

    def close_clock(self):
        """ Closes the clock and cleans up afterwards.

        @return int: error code (0:OK, -1:error)

        TODO: This method is very hardware specific, it should be deprecated
        """
        return 0

    """
    # ========================================
    #    Helper functions for Counting
    # ========================================
    """

    def _get_count_rate_dll(self):
        """ Returns the current counts per second of the counter.

        @return float: the photon counts per second

        first counting method (worst): we use get_count_rate not a good dynamic, and exp time is fixed to 100ms
        """
        time.sleep(0.1)
        count_rate = np.zeros((1, 1))
        count_rate[0, 0] = self._get_count_rate(self._count_channel)
        return count_rate

    def _get_counter_hist(self):
        """ Returns the current counts per second of the counter.

        @return float: the photon counts per second

        second counting method :taking acquisition time=1/freq
        and summing get_histogram acquired on this acquisition time
        """
        count_rate = np.zeros((1, 1))

        self._clear_hist_memory()
        self.meas_run = True

        exposure_time = 1 / self._clock_frequency
        self._start(int(exposure_time * 1000))

        ctc_status = 0
        while ctc_status != 1:
            ctc_status = int(self._get_ctc_status())

        self._stop_meas()
        self.meas_run = False

        count_rate[0, 0] = sum(self._get_histogram(self._count_channel))/self.exposure_time
        return count_rate

    """
    ====================================================================================================================
                                                            API
    ====================================================================================================================
    """


    """
    # ========================================
    #        Hardware Helper functions
    # ========================================
    """

    def _create_error_code(self):
        """ Create a dictionary with the error code for the device.

        @return dict: error code in a dictionary.

        The error code is extracted of MHLib Ver. 3.1, March 2022.
        It can be also extracted by calling the _get_error_string method with the appropriate integer value.
        """
        # FIXME: path: To do properly!
        path = r'D:\qudisoftware\qudi-iqo-modules\src\qudi\hardware\picoquant\mherrorcodes.h'
        filename = os.path.join(path)
        try:
            with open(filename) as f:
                content = f.readlines()
        except:
            self.log.error('No file "mherrorcodes.h" could be found in the MultiHarp hardware directory!')

        error_code = {}
        for line in content:
            if '#define ERROR' in line:
                error_string, error_value = line.split()[-2:]
                error_code[int(error_value)] = error_string
        return error_code

    def _set_constants(self):
        """ Set the constants (max and min values) for the MultiHarp150 device.
        These setting are taken from mhdefin.h. """

        # Max number of USB devices:
        self._max_device_number = 8
        # Max number of physical input channels:
        self._max_input_channel = 64
        # Max number of binning steps, get actual number via _get_base_resolution():
        self._max_bin_steps = 24
        # Max number of histogram bins:
        self._max_histogram_len = 65536
        # Number of event records that can be read by _tttr_read_fifo,
        # buffer must provide space for this number of dwords (1 Mo):
        self._tt_read_max = 1048576

        # Symbolic constants for MH_Initialize:
        self.MODE_HIST = 0
        self.MODE_T2 = 2
        self.MODE_T3 = 3

        # Limits for _set_histo_len. Length codes 0 and 1 will not work with _get_histogram.
        # If you need these short lengths then use _get_all_histograms.
        self._min_len_code = 0
        self._max_len_code = 6  # default

        # Limits for _set_sync_div.
        self._min_sync_div = 1
        self._max_sync_div = 16

        # Limits for _set_sync_edge_trg and _set_input_edge_trg.
        self._min_trigger_lvl = -1200  # mV
        self._max_trigger_lvl = 1200  # mV

        # Limits for _set_sync_channel_offset and _set_input_channel_offset.
        self._min_channel_offset = -99999  # ps
        self._max_channel_offset = 99999  # ps

        # Limits for _set_sync_dead_time and _set_input_dead_time.
        self.EXTDEADMIN = 800  # ps
        self.EXTDEADMAX = 160000  # ps

        # Limits for _set_offset.
        self._min_offset = 0  # ns
        self._max_offset = 100000000  # ns

        # Limits for _start.
        self.ACQTMIN = 1  # ms
        self.ACQTMAX = 360000000  # ms  (100*60*60*1000ms = 100h)

        # Limits for _set_stop_overflow.
        self.STOPCNTMIN = 1
        self.STOPCNTMAX = 4294967295  # 32 bit is mem max

        # Limits for _set_trigger_output.
        self.TRIGOUTMIN = 0  # 0=off
        self.TRIGOUTMAX = 16777215  # in units of 100ns

        # Limits for _tttr_set_marker_holdofftime.
        self.HOLDOFFMIN = 0  # ns
        self.HOLDOFFMAX = 25500  # ns

        # Limits for _set_input_hysteresis.
        self.HYSTCODEMIN = 0  # approx. 3mV
        self.HYSTCODEMAX = 1  # approx. 35mV

        # Limits for _tttr_set_ofl_compression.
        self.HOLDTIMEMIN = 0  # ms
        self.HOLDTIMEMAX = 255  # ms

        # Limits for MH_SetRowEventFilterXXX and MH_SetMainEventFilter.
        self.ROWIDXMIN = 0
        self.ROWIDXMAX = 8  # actual upper limit is smaller, dep. on rows present
        self.MATCHCNTMIN = 1
        self.MATCHCNTMAX = 6
        self.INVERSEMIN = 0
        self.INVERSEMAX = 1
        self.TIMERANGEMIN = 0  # ps
        self.TIMERANGEMAX = 160000  # ps
        self.USECHANSMIN = 0x000  # no channels used
        self.USECHANSMAX = 0x1FF  # note: sync bit 0x100 will be ignored in T3 mode and in row filter
        self.PASSCHANSMIN = 0x000  # no channels passed
        self.PASSCHANSMAX = 0x1FF  # note: sync bit 0x100 will be ignored in T3 mode and in row filter

    def _check(self, func_val):
        """ Check routine for the received error codes.

        @param int func_val: return error code of the called function.
        @return int: pass the error code further so that other functions have the possibility to use it.

        Each called function in the dll has an 32-bit return integer, which indicates, whether the function was called
        and finished successfully (then func_val = 0) or if any error has occurred (func_val < 0).
        The error code, which corresponds to the return value can be looked up in the file 'mherrorcodes.h'.
        """
        if not func_val == 0:
            self.log.error('Error in MultiHarp150 with error code {0}:\n'
                           '{1}'.format(func_val, self.error_code[func_val]))
        return func_val

    """
    # ========================================
    #           Hardware functions
    # ========================================
    """
    # These 2 next functions work independent of any device.
    # ------------------------------------------------------

    def _get_version(self):
        """ Get the software/library version of the device.

        @return str: the version number of the current library.
        """
        lib_version = ctypes.create_string_buffer(8)
        self._check(self._dll.MH_GetLibraryVersion(ctypes.byref(lib_version)))
        return lib_version.value.decode()  # .decode() converts byte to string.

    def _get_error_string(self, errcode):
        """ Get the string error code from the MultiHarp Device.

        @param int errcode: error code from 0 and below.
        @return str: description of the error code.

        The string code for the error is the same as it is extracted from the mherrorcodes.h header file.
        Note that errcode should have the value 0 or lower, since integer bigger 0 are not defined as error.
        """
        err_string = ctypes.create_string_buffer(40)
        self._check(self._dll.MH_GetErrorString(ctypes.byref(err_string), errcode))
        return err_string.value.decode()

    # Functions to establish the connection and initialize the device or disconnect it.
    # ---------------------------------------------------------------------------------

    def _open_connection(self):
        """ Open a connection to this device. """
        hw_serial = ctypes.create_string_buffer(8)
        ret = self._check(self._dll.MH_OpenDevice(self._deviceID, ctypes.byref(hw_serial)))
        if ret >= 0:
            self.connected_to_device = True
            self.log.info('Connection to the MultiHarp 150 established')

    def _close_connection(self):
        """Close the connection to the device. """
        self.connected_to_device = False
        self._check(self._dll.MH_CloseDevice(self._deviceID))
        self.log.info('Connection to the MultiHarp 150 closed.')

    def _initialize(self, mode, ref_source=0):
        """ Initialize the device with one of the three possible modes.

        @param int mode:    0: histogramming
                            2: T2
                            3: T3
        @param int ref_source: 0: use internal clock
                              1: use 10 MHz external clock
                              2: White Rabbit master with generic partner
                              3: White Rabbit slave with generic partner
                              4: White Rabbit grand master with generic partner
                              5: use 10 MHz + PPS from GPS receiver
                              6: use 10 MHz + PPS + time via UART from GPS receiver
                              7: White Rabbit master with MultiHarp as partner
                              8: White Rabbit slave with MultiHarp as partner
                              9: White Rabbit grand master with MultiHarp as partner

        Note: selecting WR as a clock source requires that a WR connection has actually been established beforehand.
        Unless the WR connection is established by a WR startup script this will require a two stage process initially
        initializing with internal clock source, then setting up the WR connection by means of the WR routines
        described below, then initializing again with the desired WR clock mode.
        """
        mode = int(mode)  # convert to integer for safety reasons.
        self._mode = mode
        ref_source = int(ref_source)
        self._ref_source = ref_source

        if not ((mode != self.MODE_HIST) or (mode != self.MODE_T2) or (mode != self.MODE_T3)):
            self.log.error(f'MultiHarp: Mode for the device could not be set. '
                           f'It must be {self.MODE_HIST}=Histogram-Mode, {self.MODE_T2}=T2-Mode or '
                           f'{self.MODE_T3}=T3-Mode, but a parameter {mode} has been passed.')
        else:
            self._check(self._dll.MH_Initialize(self._deviceID, mode, ref_source))

    # All functions below can be used if the device was successfully called.
    # ----------------------------------------------------------------------

    def _get_base_resolution(self):
        """ Retrieve the base resolution of the device.

        @return string tuple(2): (Base resolution in ps, Number of allowed binning steps).

        Note: The base resolution of a device is its best possible resolution as determined by the hardware.
        It also corresponds to the timing resolution in T2 mode.
        In T3 and Histogramming mode it is possible to “bin down” the resolution (see MH_SetBinning).
        The value returned in binsteps is the number of permitted binning steps.
        The range of values you can pass to MH_SetBinning is then 0 to binsteps-1.
        """
        resolution = ctypes.c_double()
        binsteps = ctypes.c_int32()
        self._check(self._dll.MH_GetBaseResolution(self._deviceID, ctypes.byref(resolution), ctypes.byref(binsteps)))
        return resolution.value, binsteps.value

    def _get_number_of_input_channels(self):
        """ Retrieve the number of installed input channels.

        @return int: the number of channels.

        Note: The range of values you can pass to the library calls accepting a channel number is then 0 to nchannels-1.
        """
        nchannels = ctypes.c_int32()
        self._check(self._dll.MH_GetNumOfInputChannels(self._deviceID, ctypes.byref(nchannels)))
        return nchannels.value

    def _set_sync_div(self, div):
        """ Set the synchronization channel divider of the device.

        @param int div: synchronization rate divider (1, 2, 4, 8, _max_sync_div).

        Note: The sync divider must be used to keep the effective sync rate at values < 78 MHz.
        It should only be used with sync sources of stable period. Using a larger divider
        than strictly necessary does not do great harm, but it may result in slightly larger timing jitter.
        The readings obtained with MH_GetCountRate are internally corrected for the divider setting and
        deliver the external (undivided) rate. The sync divider should not be changed while a measurement is running.
        """
        if not ((div != self._min_sync_div) or (div != 2) or (div != 4) or (div != 8) or (div != self._max_sync_div)):
            self.log.error(f'MultiHarp: Invalid synchronization divider. '
                           f'Value must be 1, 2, 4, 8 or 16, but a value of {div} has been passed.')
            return -1
        else:
            self._check(self._dll.MH_SetSyncDiv(self._deviceID, div))

    def _set_sync_edge_trg(self, level, edge):
        """ Set the synchronization channel trigger level.

        @param int level: trigger level in mV, between _min_trigger_lvl to _max_trigger_lvl.
        @param int edge: 0 = falling, 1 = rising.

        Note: The hardware uses a 10 bit DAC that can resolve the level value only in steps of about 2.34 mV.
        """
        if not (self._min_trigger_lvl <= level <= self._max_trigger_lvl):
            self.log.error(f'MultiHarp: Invalid synchronization trigger level. Value must be within the range '
                           f'[{self._min_trigger_lvl},{self._max_trigger_lvl}] mV, but a value of {level} has been passed.')
            return -1
        else:
            self._check(self._dll.MH_SetSyncEdgeTrg(self._deviceID, level, edge))

    def _set_sync_channel_offset(self, offset):
        """ Set the synchronization channel offset.

        @param int offset: offset (time shift) in ps for that channel, between _min_channel_offset to _max_channel_offset.
        """
        offset = int(offset)
        if not (self._min_channel_offset <= offset <= self._max_channel_offset):
            self.log.error(f'MultiHarp: Invalid synchronization offset. Value must be within the range '
                           f'[{self._min_channel_offset},{self._max_channel_offset}] ps, but a value of {offset} has been passed.')
            return -1
        else:
            self._check(self._dll.MH_SetSyncChannelOffset(self._deviceID, offset))

    def _set_input_edge_trg(self, channel, level, edge):
        """ Set the input channel trigger level.

        @param int channel: input channel index (0 to nchannels-1).
        @param int level: trigger level in mV, between _min_trigger_lvl to _max_trigger_lvl.
        @param int edge: 0 = falling, 1 = rising.
        """
        if not (self._min_trigger_lvl <= level <= self._max_trigger_lvl):
            self.log.error(f'MultiHarp: Invalid input trigger level. Value must be within the range '
                           f'[{self._min_trigger_lvl},{self._max_trigger_lvl}] mV, but a value of {level} has been passed.')
            return -1
        else:
            self._check(self._dll.MH_SetInputEdgeTrg(self._deviceID, channel, level, edge))

    def _set_input_channel_offset(self, channel, offset):
        """ Set the input channel offset.

        @param int channel: input channel index (0 to nchannels-1).
        @param int offset: channel timing offset in ps, between _min_channel_offset to _max_channel_offset.
        """
        offset = int(offset)
        if not (self._min_channel_offset <= offset <= self._max_channel_offset):
            self.log.error(f'MultiHarp: Invalid input offset. Value must be within the range '
                           f'[{self._min_channel_offset},{self._max_channel_offset}] ps, but a value of {offset} has been passed.')
            return -1
        else:
            self._check(self._dll.MH_SetInputChannelOffset(self._deviceID, channel, offset))

    def _set_input_channel_enable(self, channel, enable):
        """ Enable or disable the specified input channel.

        @param int channel: input channel index (0 to nchannels-1).
        @param int enable: desired enable state of the input channel (0 = disabled, 1 = enabled).
        """
        self._check(self._dll.MH_SetInputChannelEnable(self._deviceID, channel, enable))

    def _set_input_dead_time(self, channel, on, deadtime):
        """ Set the input channel dead time for the specified channel.
        Primarily intended for the suppression of afterpulsing artefacts of some detectors.
        The actual extended dead time is only approximated to the nearest step of the device’s base resolution.

        @param int channel: input channel index (0 to nchannels-1).
        @param int on: 0 = set minimal dead-time, 1 = activate extended dead-time.
        @param int deadtime: extended dead-time in ps, between EXTDEADMIN to EXTDEADMAX.
        """
        if not (self.EXTDEADMIN <= deadtime <= self.EXTDEADMAX):
            self.log.error(f'MultiHarp: Invalid input deadtime. Value must be within the range '
                           f'[{self.EXTDEADMIN},{self.EXTDEADMAX}] ps, but a value of {deadtime} has been passed.')
            return -1
        else:
            self._check(self._dll.MH_SetInputDeadTime(self._deviceID, channel, on, deadtime))

    def _set_binning(self, binning):
        """ Set the base resolution of the measurement.

        @param int binning: binning code, between 0 (smallest, i.e. base resolution) to (_max_bin_steps-1) (largest).
        The binning code corresponds to a power of 2, i.e.
            0 =   base resolution,        => 5*2^0 =    5ps
            1 =   2x base resolution,     => 5*2^1 =   10ps
            2 =   4x base resolution,     => 5*2^2 =   20ps
            3 =   8x base resolution      => 5*2^3 =   40ps
            4 =  16x base resolution      => 5*2^4 =   80ps
            5 =  32x base resolution      => 5*2^5 =  160ps
            6 =  64x base resolution      => 5*2^6 =  320ps
            7 = 128x base resolution      => 5*2^7 =  640ps
            and so on.

        These are all the possible values. In histogram mode the internal buffer can store 65535 points
        (each a 32bit word). For largest resolution you can count 33.55392 ms in total.
        """
        if not (0 <= binning < self._max_bin_steps):
            self.log.error(f'MultiHarp: Invalid binning. Value must be within the range '
                           f'[0,{self._max_bin_steps}] bins, but a value of {binning} has been passed.')
            return -1
        else:
            self._check(self._dll.MH_SetBinning(self._deviceID, binning))

    def _set_histo_len(self, len_code):
        """ Set the length of histograms.

        @param int len_code: histogram length code, between 0 to _max_len_code (default).
        @return int: current length (time bin count) of histograms.
                     It calculates as 1024 times len_code to the power of 2.

        Note: This sets the number of bins of the collected histograms. The histogram length obtained with
        _max_len_code is 65536 which is also the default after initialization if MH_SetHistoLen is not called.
        """
        actual_len = ctypes.c_int()
        if not (self._min_len_code <= len_code <= self._max_len_code):
            self.log.error(f'MultiHarp: Invalid len_code.\nValue must be within the range '
                           f'[{self._min_len_code},{self._max_len_code}], but a value of {len_code} has been passed.')
            return -1
        else:
            self._check(self._dll.MH_SetHistoLen(self._deviceID, len_code, ctypes.byref(actual_len)))
        return actual_len.value

    def _clear_hist_memory(self):
        """ Clear the histogram memory of all channels. Only meaningful in histogramming mode.
        """
        self._check(self._dll.MH_ClearHistMem(self._deviceID))

    def _start(self, acq_time):
        """ Start acquisition for the specified time (in ms).

        @param int acq_time: acquisition time in milliseconds, between ACQTMIN to ACQTMAX.

        Note: For the 'MEASCTRL_SW_START_SW_STOP' measurement control mode, the parameter acq_time will be ignored and
        the measurement will run until MH_StopMeas (_stop_meas) is called.
        """
        if not (self.ACQTMIN <= acq_time <= self.ACQTMAX):
            self.log.error(f'MultiHarp: No measurement could be started. The acquisition time must be within the range'
                           f' [{self.ACQTMIN},{self.ACQTMAX}] ms, but a value of {acq_time} has been passed.')
            return -1
        else:
            self._check(self._dll.MH_StartMeas(self._deviceID, int(acq_time)))

    def _stop_meas(self):
        """ Stop the measurement. """
        self._check(self._dll.MH_StopMeas(self._deviceID))

    def _get_ctc_status(self):
        """ Check the status of the device.

        @return int: 0: acquisition time still running, 1: acquisition time has ended, measurement finished.
        """
        ctc_status = ctypes.c_int()
        self._check(self._dll.MH_CTCStatus(self._deviceID, ctypes.byref(ctc_status)))
        return ctc_status.value

    def _get_histogram(self, channel: int) -> np.ndarray:
        """ Retrieve the measured histogram of the specified channel.

        @param int channel: input channel index, between 0 to nchannels-1.
        @return np.ndarray ch_count: array of histogram length size corresponding to the histogram data.

        Note: This function cannot be used with the shortest two histogram lengths of 1024 and 2048 bins.
        You need to use MH_GetAllHistograms in this case.
        """
        actual_length = self._set_histo_len(self._len_code)
        ch_count = np.ones((actual_length,), dtype=np.uint32)
        channel = int(channel)
        channel_max = self._get_number_of_input_channels()
        if not (0 <= channel <= channel_max):
            self.log.error(f'MultiHarp: Invalid channel number. Channel number must be an integer between'
                           f' 0 to 3, but {channel} has been passed.')
            return -1
        self._dll.MH_GetHistogram.argtypes = [ctypes.c_int32, ctypes.c_int64, ctypes.c_int32]
        self._check(self._dll.MH_GetHistogram(self._deviceID, ch_count.ctypes.data, channel))
        return ch_count

    def _get_count_rate(self, channel):
        """ Get the current count rate for the input channel.

        @param int channel: input channel to read.
        @return int count_rate: count rate in s^-1.

        Note: Allow at least 100 ms after MH_Initialize to get a stable rate meter reading.
        Similarly, wait at least 100ms to get a new reading. This is the gate time of the counters.
        The maximum input channel index must correspond to nchannels-1 as obtained through MH_GetNumOfInputChannels().
        """
        if not ((channel != 0) or (channel != 1) or (channel != 2) or (channel != 3)):
            self.log.error(f'MultiHarp: Count rate could not be read out, channel does not exist. '
                           f'Channel has to be 0, 1, 2 or 3, but {channel} has been passed.')
            return -1
        else:
            count_rate = ctypes.c_int()
            self._check(self._dll.MH_GetCountRate(self._deviceID, channel, ctypes.byref(count_rate)))
            return count_rate.value

    """
    NO USAGE
    """

    def _get_hardware_info(self):
        """ Retrieve the device hardware information.

        @return string tuple(3): (hw_model, hw_part_num, hw_version).
        """
        hw_model = ctypes.create_string_buffer(24)
        hw_version = ctypes.create_string_buffer(8)
        hw_part_num = ctypes.create_string_buffer(8)
        self._check(self._dll.MH_GetHardwareInfo(self._deviceID, ctypes.byref(hw_model),
                                                 ctypes.byref(hw_part_num), ctypes.byref(hw_version)))
        # the .decode() function converts byte objects to string objects
        return hw_model.value.decode(), hw_part_num.value.decode(), hw_version.value.decode()

    def _get_features(self):
        """ Retrieve the possible features of the device.

        @return int: a bit pattern indicating the feature.
        """
        features = ctypes.c_int32()
        self._check(self._dll.MH_GetFeatures(self._deviceID, ctypes.byref(features)))
        return features.value

    def _get_serial_number(self):
        """ Retrieve the serial number of the device.

        @return string: serial number of the device
        """
        serial_num = ctypes.create_string_buffer(16)  # at least 8 byte
        self._check(self._dll.MH_GetSerialNumber(self._deviceID, ctypes.byref(serial_num)))
        return serial_num.value.decode()

    def _get_num_of_modules(self):
        """ Retrieve the number of installed modules.

        @return int: the number of modules.

        Note: This routine is only an accessory for retrieval of hardware version details via
        MH_GetModuleInfo which must be called separately for each module.
        The range of values you can pass to MH_GetModuleInfo is then 0 to nummod-1.
        """
        nummod = ctypes.c_int32()
        self._check(self._dll.MH_GetNumOfModules(self._deviceID, ctypes.byref(nummod)))
        return nummod.value

    def _get_module_info(self, modidx):
        """ Retrieve the hardware version details for each module.

        @params int modidx: module index, between 0 to nummod-1.
        @return string tuple(2): (Model of the module identified by modidx,
                                  Version code of the module identified by modidx).

        Note: It must be called separately for each module. Get the number of
        modules via MH_GetNumOfModules. You only need this information for support enquiries.
        """
        model_code = ctypes.c_int()
        version_code = ctypes.c_int()
        self._check(self._dll.MH_GetModuleInfo(self._deviceID, modidx, ctypes.byref(model_code),
                                               ctypes.byref(version_code)))
        return model_code.value, version_code.value

    def _get_debug_info(self):
        """ Retrieve the debug information for the current hardware.

        @return char[65536]: the information for debugging.
        """
        debug_info = ctypes.create_string_buffer(65536)
        self._check(self._dll.MH_GetDebugInfo(self._deviceID, debug_info))
        return debug_info.value.decode()

    def _set_sync_channel_enable(self, enable):
        """ Enable or disable the synchronization channel.
        Only useful in T2 mode. Histogramming and T3 mode need an active sync signal.

        @param int enable: desired enable state of the sync channel (0: disabled, 1: enabled).
        """
        self._check(self._dll.MH_SetSyncChannelEnable(self._deviceID, enable))

    def _set_sync_dead_time(self, on, deadtime):
        """ Set the sync channel dead time.
        Primarily intended for the suppression of afterpulsing artefacts of some detectors.
        The actual extended dead time is only approximated to the nearest step of the device’s base resolution.

        @param int on: 0 = set minimal dead-time, 1 = activate extended dead-time.
        @param int deadtime: extended dead-time in ps, between EXTDEADMIN to EXTDEADMAX.
        """
        if not (self.EXTDEADMIN <= deadtime <= self.EXTDEADMAX):
            self.log.error(f'MultiHarp: Invalid synchronization deadtime. Value must be within the range '
                           f'[{self.EXTDEADMIN},{self.EXTDEADMAX}] ps, but a value of {deadtime} has been passed.')
            return -1
        else:
            self._check(self._dll.MH_SetSyncDeadTime(self._deviceID, on, deadtime))

    def _set_input_hysteresis(self, hyst_code):
        """ Set the hysteresis value for all the timing inputs (sync and channels) simultaneously. Intended for the
        suppression of noise or pulse shape artefacts of some detectors by setting a higher input hysteresis.

        @param int hyst_code: code for the hysteresis (0 = 3mV approx. (default), 1 = 35mV approx.).
        """
        if hyst_code not in (self.HYSTCODEMIN, self.HYSTCODEMAX):
            self.log.error(f'MultiHarp: Invalid hyst_code parameter. The hyst_code parameter must be either '
                           f'{self.HYSTCODEMIN} or {self.HYSTCODEMAX}, but a value of {hyst_code} has been passed.')
            return -1
        else:
            self._check(self._dll.MH_SetInputHysteresis(self._deviceID, hyst_code))

    def _set_stop_overflow(self, stop_overflow, stop_count):
        """ Stop the measurement if maximal amount of counts is reached.

        @param int stop_overflow: 0 = do not stop, 1 = do stop on overflow.
        @param int stop_count: count level at which should be stopped, between STOPCNTMIN to STOPCNTMAX.

        This setting determines if a measurement run will stop if any channel reaches the maximum set by stop_count.
        If stop_ofl is 0 the measurement will continue but counts above STOPCNTMAX in any bin will be clipped.
        """
        if stop_overflow not in (0, 1):
            self.log.error(f'MultiHarp: Invalid overflow parameter. The overflow parameter must be either '
                           f'0 or 1 but a value of {stop_overflow} has been passed.')
            return -1
        if not (self.STOPCNTMIN <= stop_count <= self.STOPCNTMAX):
            self.log.error(f'MultiHarp: Invalid stop_count parameter. Value must be within the range '
                           f'[{self.STOPCNTMIN},{self.STOPCNTMAX}], but a value of {stop_count} has been passed.')
            return -1
        else:
            self._check(self._dll.MH_SetStopOverflow(self._deviceID, stop_overflow, stop_count))

    def _set_offset(self, offset):
        """ Set an offset time.

        @param int offset: offset in ps (only possible for histogramming and T3-mode!), between _min_offset to _max_offset.

        The true offset is an approximation for the desired offset by the nearest multiple of the base resolution.
        This offset only acts on the difference between ch1 and ch0 in histogramming and T3 mode.
        To not confuse with the input offsets!
        """
        if not (self._min_offset <= offset <= self._max_offset):
            self.log.error(f'MultiHarp: Invalid offset. Value must be within the range '
                           f'[{self._min_offset},{self._max_offset}] ns, but a value of {offset} has been passed.')
            return -1
        else:
            self._check(self._dll.MH_SetOffset(self._deviceID, offset))

    def _set_meas_control(self, meas_control, start_edge, stop_edge):
        """ Set the measurement control mode and must be called before starting a measurement.

        @param int meas_control: measurement control code, with:
                                0 = MEASCTRL_SINGLESHOT_CTC (default after initialization),
                                1 = MEASCTRL_C1_GATED,
                                2 = MEASCTRL_C1_START_CTC_STOP,
                                3 = MEASCTRL_C1_START_C2_STOP,
                                4 = MEASCTRL_WR_M2S,
                                5 = MEASCTRL_WR_S2M,
                                6 = MEASCTRL_SW_START_SW_STOP.
        @param int start_edge: start edge selection code (0 = falling, 1 = rising).
        @param int stop_edge: stop edge selection code (0 = falling, 1 = rising).
        """
        self._check(self._dll.MH_SetMeasControl(self._deviceID, meas_control, start_edge, stop_edge))

    def _set_trigger_output(self, period):
        """ Set the period of the programmable trigger output.

        @param int period: in units of 100ns, between TRIGOUTMIN to TRIGOUTMAX, with 0 = off.

        Note: This can be used to set the period of the programmable trigger output.
        The period 0 switches it off. Observe laser safety when using this feature for triggering a laser.
        """
        if not (self.TRIGOUTMIN <= period <= self.TRIGOUTMAX):
            self.log.error(f'MultiHarp: Invalid period trigger output. Value of period must be within the range '
                           f'[{self.TRIGOUTMIN},{self.TRIGOUTMAX}] 100ns, but a value of {period} has been passed.')
            return -1
        else:
            self._check(self._dll.MH_SetTriggerOutput(self._deviceID, period))

    def _get_all_histograms(self):
        """ Retrieve all the measured histogram.

        @return array of (histogram length * number of channels) size: all histogram data.
        """
        histogram_len = self._set_histo_len(self._len_code)
        num_channels = self._get_number_of_input_channels()
        counts = (ctypes.c_uint * histogram_len * num_channels)()
        self._check(self._dll.MH_GetAllHistograms(self._deviceID, ctypes.byref(counts)))
        histogram_by_channels = []
        for i in range(0, num_channels):
            histogram_by_channels.append(counts[i][:])
        return histogram_by_channels

    def _get_resolution(self):
        """ Retrieve the current resolution of the multiharp. Not meaningful in T2 mode.

        @return double: resolution at current binning in ps.
        """
        resolution = ctypes.c_double()
        self._check(self._dll.MH_GetResolution(self._deviceID, ctypes.byref(resolution)))
        return resolution.value

    def _get_sync_rate(self):
        """ Get the current count rate for the sync channel.

        @return int: sync rate.

        Note: Allow at least 100 ms after MH_Initialize or MH_SetSyncDiv to get a stable rate meter reading.
        Similarly, wait at least 100 ms to get a new reading. This is the gate time of the counter.
        """
        sync_rate = ctypes.c_int()
        self._check(self._dll.MH_GetSyncRate(self._deviceID, ctypes.byref(sync_rate)))
        return sync_rate.value

    def _get_all_count_rates(self):
        """ Get the count rates of all the channels (sync and inputs).

        @return tuple (sync_rate, count_rates):
                sync_rate: the count rates of the sync channel.
                count_rates: the count rates of the input channels.

        """
        sync_rate = ctypes.c_int()
        num_channels = self._get_number_of_input_channels()
        count_rates = (ctypes.c_uint * num_channels)()
        self._check(self._dll.MH_GetAllCountRates(self._deviceID, ctypes.byref(sync_rate), ctypes.byref(count_rates)))
        return sync_rate.value, count_rates[:]

    def _get_flags(self):
        """ Get the current status flag as a bit pattern.

        @return int: the current status flags (a bit pattern).

        Use the predefined bit mask values in phdefin.h (e.g. FLAG_OVERFLOW)
        to extract individual bits though a bitwise AND.
        """
        flags = ctypes.c_int32()
        self._check(self._dll.MH_GetFlags(self._deviceID, ctypes.byref(flags)))
        return flags.value

    def _get_elapsed_meas_time(self):
        """ Retrieve the elapsed measurement time in ms. Gives the current measurement time when still running
        or the previous measurement time when already finished.

        @return double: the elapsed measurement time in ms.
        """
        elapsed_meas_time = ctypes.c_double()
        self._check(self._dll.MH_GetElapsedMeasTime(self._deviceID, ctypes.byref(elapsed_meas_time)))
        return elapsed_meas_time.value

    def _get_start_time(self):  # Untested
        """ Retrieve the start time of a measurement with picoseconds resolution, with:
        timedw2: most significant dword of the time value.
        timedw1: 2nd most significant dword of the time value.
        timedw0: least significant dword of the time value.
        The result is to be interpreted in the sense of a unix time, i.e. elapsed picoseconds since
        January 1st 1970 00:00:00 UTC (Universal Time).

        @return int start_time: the start time of a measurement with picoseconds resolution.
        """
        timedw2 = ctypes.c_uint()
        timedw1 = ctypes.c_uint()
        timedw0 = ctypes.c_uint()
        self._check(self._dll.MH_GetStartTime(self._deviceID, ctypes.byref(timedw2),
                                              ctypes.byref(timedw1), ctypes.byref(timedw0)))
        timedw2_int = timedw2.value
        timedw1_int = timedw1.value
        timedw0_int = timedw0.value
        picoseconds = (timedw2_int << 64) | (timedw1_int << 32) | timedw0_int
        start_time = Fraction(picoseconds, 1000000000000)
        return start_time

    def _get_warnings(self):
        """Retrieve any warnings about the device or the current measurement.

        @return int warnings: a bitmask for the warnings (see mhdefin.h).

        NOTE: you have to call either MH_GetAllCountRates or call MH_GetSyncRate and MH_GetCountRate for all channels
        prior to this call! Otherwise, the received warnings will at least partially not be meaningful.
        """
        warnings = ctypes.c_int32()
        self._check(self._dll.MH_GetWarnings(self._deviceID, ctypes.byref(warnings)))
        return warnings.value

    def _get_warnings_text(self, warning_num):
        """Retrieve the warning text for the corresponding warning bitmask.

        @param int warning_num: the number for which you want to have the warning text.
        @return char[16384] text: the actual text of the warning.
        """
        text = ctypes.create_string_buffer(16284)
        self._check(self._dll.MH_GetWarningsText(self._deviceID, ctypes.byref(text), warning_num))
        return text.value.decode()

    def _get_sync_period(self):
        """ Retrieve the sync period.

        @return double: sync period in seconds

        Note: This call only gives meaningful results while a measurement is running and after two sync periods
        have elapsed. The return value is undefined in all other cases. Resolution is that of the device’s base
        resolution. Accuracy is determined by single shot jitter and clock stability.
        """
        period = ctypes.c_double()
        self._check(self._dll.MH_GetSyncPeriod(self._deviceID, ctypes.byref(period)))
        return period.value

    # ----------------------------------------------------
    # Special functions for Time-Tagged Time Resolved mode
    # ----------------------------------------------------
    # To check whether you can use the TTTR mode (must be purchased in addition) you can call MH_GetFeatures to check.

    def _tttr_read_fifo(self):
        """ Read out the buffer of the FIFO.

        @return tuple (buffer, actual_num_counts):
                buffer: data array where the TTTR data are stored (pointer to an array of _tt_read_max dwords (32bit)).
                actual_num_counts: the number of TTTR records received, maximum is _tt_read_max.

        Note: The call will return typically after 10 ms and even less if no more data could be fetched.
        The call may occasionally take longer due to USB overhead and operating system latencies, especially when
        the PC or the USB connection is slow. Buffer must not be accessed until the call returns.
        """
        num_counts = self._tt_read_max
        buffer = np.zeros((num_counts,), dtype=np.uint32)
        actual_num_counts = ctypes.c_int32()
        self._check(self._dll.MH_ReadFiFo(self._deviceID, buffer.ctypes.data,
                                          num_counts, ctypes.byref(actual_num_counts)))
        return buffer, actual_num_counts.value

    def _tttr_set_marker_edges(self, me0, me1, me2, me3):
        """ Set the marker edges.
        Note: This can be used to enable or disable the external TTL marker inputs (nly meaningful in TTTR mode).

        @param int me0: active edge of marker signal 0, with 0 = falling and 1 = rising.
        @param int me1: active edge of marker signal 1, with 0 = falling and 1 = rising.
        @param int me2: active edge of marker signal 2, with 0 = falling and 1 = rising.
        @param int me3: active edge of marker signal 3, with 0 = falling and 1 = rising.
        """
        if me0 not in (0, 1) or me1 not in (0, 1) or me2 not in (0, 1) or me3 not in (0, 1):
            self.log.error(f'MultiHarp: Invalid value. Marker edges must be either 0 or 1, but the current marker'
                           f' settings have been passed: me0={me0}, me1={me1}, me2={me2}, me3={me3}.')
            return -1
        else:
            self._check(self._dll.MH_SetMarkerEdges(self._deviceID, me0, me1, me2, me3))

    def _tttr_set_marker_enable(self, me0, me1, me2, me3):
        """ Set the marker enable or not.

        @param int me0: desired enable state of marker signal 0, with 0 = disabled, 1 = enabled.
        @param int me1: desired enable state of marker signal 1, with 0 = disabled, 1 = enabled.
        @param int me2: desired enable state of marker signal 2, with 0 = disabled, 1 = enabled.
        @param int me3: desired enable state of marker signal 3, with 0 = disabled, 1 = enabled.
        """
        if me0 not in (0, 1) or me1 not in (0, 1) or me2 not in (0, 1) or me3 not in (0, 1):
            self.log.error(f'MultiHarp: Invalid value. Marker enable options must be either 0 or 1, but the current'
                           f' marker settings have been passed: me0={me0}, me1={me1}, me2={me2}, me3={me3}.')
            return -1
        else:
            self._check(self._dll.MH_SetMarkerEnable(self._deviceID, me0, me1, me2, me3))

    def _tttr_set_marker_holdofftime(self, hold_off_time):
        """ Set the hold-off time for the markers.

        @param int hold_off_time: hold-off time in ns, between 0 to HOLDOFFMAX.

        This setting can be used to clean up glitches on the marker signals. When set to X ns then after detecting
        a first marker edge the next marker will not be accepted before x ns. Observe that the internal granularity
        of this time is only about 50ns. The hold-off time is set equally for all marker inputs
        but the hold off logic acts on each marker independently.
        """
        if not (self.HOLDOFFMIN <= hold_off_time <= self.HOLDOFFMAX):
            self.log.error(f'MultiHarp: Invalid value. Hold-off time value must be within the range [{self.HOLDOFFMIN},'
                           f' {self.HOLDOFFMAX}] ns, but a value of {hold_off_time} has been passed.')
            return -1
        else:
            self._check(self._dll.MH_SetMarkerHoldoffTime(self._deviceID, hold_off_time))

    def _tttr_set_ofl_compression(self, hold_time):
        """ Set overflow hold time for data compression. Useful when data rates are very low and there are
        more overflows than photons. The hardware will then count overflows and only transfer them to the FiFo
        when hold time has elapsed. The default value is 2 ms.

        @param int hold_time: hold time in ms, between 0 to HOLDTIMEMAX.

        Note: If you are implementing a real-time preview and data rates are very low, you may observe “stutter”
        when 'hold_time' is chosen too large because then there is nothing coming out of the FiFo for longer times.
        Indeed, this is aggravated by the fact that the FiFo has a transfer granularity of 16 records.
        Supposing a data stream without any regular event records (i.e. only overflows) this means that effectively
        there will be transfers only every 16*hold_time ms. Whenever there is a true event record arriving
        (photons or markers) the previously accumulated overflows will instantly be transferred.
        This may be the case merely due to dark counts, so the “stutter” would rarely occur.
        In any case, you can switch overflow compression off by setting hold_time to 0.
        """
        if not (self.HOLDTIMEMIN <= hold_time <= self.HOLDTIMEMAX):
            self.log.error(f'MultiHarp: Invalid value. Value of hold time must be within the range '
                           f'[{self.HOLDTIMEMIN},{self.HOLDTIMEMAX}] ms, but a value of {hold_time} has been passed.')
            return -1
        else:
            self._check(self._dll.MH_SetOflCompression(self._deviceID, hold_time))

    # ----------------------------------------------------
    # Special Functions for TTTR Mode with Event Filtering
    # ----------------------------------------------------
    # Event filtering helps to reduce USB bus load in TTTR mode by eliminating photon events
    # that carry no information of interest as typically found in many coincidence correlation experiments.
    # Please read the MultiHarp manual for details.

    def _tttr_set_row_event_filter(self):
        """ TO DO """
        # MH_SetRowEventFilter(int devidx, int rowidx, int timerange, int matchcnt, int inverse, int usechannels, int passchannels)
        pass

    def _tttr_enable_row_event_filter(self):
        """ TO DO """
        # MH_EnableRowEventFilter(int devidx, int rowidx, int enable)
        pass

    def _tttr_set_main_event_filter_params(self):
        """ TO DO """
        # MH_SetMainEventFilterParams(int devidx, int timerange, int matchcnt, int inverse)
        pass

    def _tttr_set_main_event_filter_channels(self):
        """ TO DO """
        # MH_SetMainEventFilterChannels(int devidx, int rowidx, int usechannels, int passchannels)
        pass

    def _tttr_enable_main_event_filter(self):
        """ TO DO """
        # MH_EnableMainEventFilter(int devidx, int enable)
        pass

    def _tttr_set_filter_test_mode(self):
        """ TO DO """
        # MH_SetFilterTestMode(int devidx, int testmode);
        pass

    def _tttr_get_row_filtered_rates(self):
        """ TO DO """
        # MH_GetRowFilteredRates(int devidx, int* syncrate, int* cntrates);
        pass

    def _tttr_get_main_filtered_rates(self):
        """ TO DO """
        # MH_GetMainFilteredRates(int devidx, int* syncrate, int* cntrates);
        pass

    # ----------------------------------
    # Special Functions for White Rabbit
    # ----------------------------------

    def _wrabbit_get_mac_address(self):
        """ Retrieve the MAC address.

        @return string: MAC address of the device.
        """
        mac_address = ctypes.create_string_buffer(12)  # 6 byte
        self._check(self._dll.MH_WRabbitGetMAC(self._deviceID, ctypes.byref(mac_address)))
        return mac_address.value.decode()

    def _wrabbit_set_mac_address(self, mac_address):
        """ Set the MAC address.

        @params string mac_address: Pointer to an array of six bytes holding the MAC address.

        Note: The MAC address must be unique, at least within the network you are using.
        """
        self._check(self._dll.MH_WRabbitSetMAC(self._deviceID, mac_address))

    def _wrabbit_get_init_script(self):
        """ TO DO """
        init_script = ctypes.create_string_buffer(512)  # at least 256 byte
        self._check(self._dll.MH_WRabbitGetInitScript(self._deviceID, ctypes.byref(init_script)))
        return init_script.value.decode()

    def _wrabbit_set_init_script(self, init_script):
        """ TO DO """
        self._check(self._dll.MH_WRabbitSetInitScript(self._deviceID, init_script))

    def _wrabbit_get_sfp_data(self):
        """ TO DO """
        # sfpnames = ctypes.create_string_buffer()
        # dTxs = ctypes.
        # dRxs = ctypes.
        # alphas = ctypes.
        # self._check(self._dll.MH_WRabbitGetSFPData(self._deviceID, ctypes.byref(sfpnames)
        # , ctypes.byref(dTxs), ctypes.byref(dRxs)
        # , ctypes.byref(alphas)))
        # return sfpnames.value.decode(), dTxs.value.decode(), dRxs.value.decode(), alphas.value.decode()
        pass

    def _wrabbit_set_sfp_data(self, sfpnames, dtxs, drxs, alphas):
        """ TO DO """
        # return.self._check(self._dll.MH_WRabbitSetSFPData(self._deviceID, sfpnames, dTxs, dRxs, alphas)
        pass

    def _wrabbit_init_link(self, link_on):
        """ TO DO

        @params int link_on: 0 = off, 1 = on
        """
        self._check(self._dll.MH_WRabbitInitLink(self._deviceID, link_on))

    def _wrabbit_set_mode(self):
        """ TO DO """
        # MH_WRabbitSetMode(int devidx, int bootfromscript, int reinit_with_mode, int mode)
        pass

    def _wrabbit_set_time(self):
        """ TO DO """
        # MH_WRabbitSetTime(int devidx, unsigned int timehidw, unsigned int timelodw)
        pass

    def _wrabbit_get_time(self):
        """ TO DO """
        # MH_WRabbitGetTime(int devidx, unsigned int* timehidw, unsigned int* timelodw,
        # unsigned int* subsec16ns)
        pass

    def _wrabbit_get_status(self):
        """ TO DO """
        # MH_WRabbitGetStatus(int devidx, int* wrstatus)
        pass

    def _wrabbit_get_term_output(self):
        """ TO DO """
        # MH_WRabbitGetTermOutput(int devidx, char* buffer, int* nchar)
        pass
