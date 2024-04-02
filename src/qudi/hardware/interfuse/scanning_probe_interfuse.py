# -*- coding: utf-8 -*-
"""
This file contains the Qudi scanning probe interfuse module for the confocal scanner.

Copyright (c) 2021, the qudi developers. See the AUTHORS.md file at the top-level directory of this
distribution and on <https://github.com/Ulm-IQO/qudi-iqo-modules/>

This file is part of qudi.

Qudi is free software: you can redistribute it and/or modify it under the terms of
the GNU Lesser General Public License as published by the Free Software Foundation,
either version 3 of the License, or (at your option) any later version.

Qudi is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with qudi.
If not, see <https://www.gnu.org/licenses/>.
"""

import time
import numpy as np
from PySide2 import QtCore
from fysom import FysomError
from qudi.core.configoption import ConfigOption
from qudi.core.connector import Connector

from qudi.interface.scanning_probe_interface import ScanningProbeInterface, ScanData
from qudi.interface.scanning_probe_interface import ScanConstraints, ScannerAxis, ScannerChannel


class ScanningProbeInterfuse(ScanningProbeInterface):
    """
    This interfuse combines modules of a scanning device and a counter device 
    to make up a scanning probe hardware. It features a counting module 
    (Picoquant Multiharp150), and a scanning module (Newport ESP300) 
    to realize 1D/2D scans.

    Example config for copy-paste:

    scanning_probe_interfuse:
        module.Class: 'scanning_probe_interfuse.ScanningProbeInterfuse'
        options:
            position_ranges:
                x: [0, 200e-6]
                y: [0, 200e-6]
                z: [-100e-6, 100e-6]
            frequency_ranges:
                x: [1, 5000]
                y: [1, 5000]
                z: [1, 1000]
            resolution_ranges:
                x: [1, 10000]
                y: [1, 10000]
                z: [2, 1000]
            position_accuracy:
                x: 50e-9
                y: 50e-9
                z: 50e-9
    """
    _modclass = 'confocalscannerinterface'
    _modtype = 'hardware'

    scanner_hardware = Connector(name='scanner_hardware', interface='MotorInterface')
    counter_hardware = Connector(name='counter_hardware', interface='SlowCounterInterface')

    # Config options
    _position_ranges = ConfigOption(name='position_ranges', missing='error')
    _frequency_ranges = ConfigOption(name='frequency_ranges', missing='error')
    _resolution_ranges = ConfigOption(name='resolution_ranges', missing='error')
    _position_accuracy = ConfigOption(name='position_accuracy', missing='error')
    
    # Signal
    sigScanFinishLine = QtCore.Signal()
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Scan process parameters
        self._current_scan_axes = tuple()
        self._current_scan_ranges = [tuple(), tuple()]
        self._current_scan_frequency = -1
        self._current_scan_resolution = tuple()
        self._current_position = dict()
        self._scan_image = None
        self._scan_data = None
        self._x_pos_before_scan = None
        self._y_pos_before_scan = None

        # "Hardware" constraints
        self._constraints = None

        # Timer parameters.
        self.__scan_start = 0
        self.__last_line = -1
        self.__update_timer = None
        
        # Start_scan parameters.
        self.last_scanned_line = []
        self.x_step = None
        self.y_step = None
        self.x_values = None
        self.y_values = None
        self.x_start_pos = None
        self.y_start_pos = None
        self.x_end_pos = None
        self.y_end_pos = None
        
        # New params for test.
        self._current_position_ranges = dict()
        self._stop_requested = False
        
    def on_activate(self):
        """ Initialisation performed during activation of the module."""
        self._scanner_hardware = self.scanner_hardware()
        self._counter_hardware = self.counter_hardware()
        
        # Set default process values
        self._current_scan_axes = tuple(self._position_ranges)[:2]
        self._current_scan_ranges = tuple(tuple(scan_range) for scan_range in tuple(self._position_ranges.values())[:2])
        self._current_scan_resolution = tuple(min(scan_res) for scan_res in self._resolution_ranges.values())[:2]
        self._current_position = self.get_position()
        self._scan_image = np.zeros(self._current_scan_resolution)
        self._scan_data = None
        
        # TODO TEST: For plotting data not exclusively centered on (0,0)?
        self._current_position_ranges = {ax: [pos + self._position_ranges[ax][0], pos + self._position_ranges[ax][1]] for ax, pos in self._current_position.items()}
        
        # TODO TEST: Added for 'stop_scan'.
        self._stop_requested = False
        
        # TODO: link with frequency of counter hardware mh150.
        self.exposure_time_s = 0.1  # TODO: Hard-coded!!
        self._current_scan_frequency = 1 / self.exposure_time_s
        
        # Generate static constraints
        axes = list()
        for ax, ax_range in self._position_ranges.items():
            dist = max(ax_range) - min(ax_range)
            axes.append(ScannerAxis(name=ax, unit='m', value_range=ax_range,
                                    step_range=(0, dist),
                                    resolution_range=self._resolution_ranges[ax],
                                    frequency_range=self._frequency_ranges[ax]))
        channels = [ScannerChannel(name='channel0', unit='c/s', dtype=np.float64)]
        self._constraints = ScanConstraints(axes=axes, channels=channels,
                                            backscan_configurable=False,
                                            has_position_feedback=False,
                                            square_px_only=False)
        
        # Setup timer.
        self.__scan_start = 0
        self.__last_line = -1
        self.__update_timer = QtCore.QTimer()
        self.__update_timer.setSingleShot(True)
        self.__update_timer.timeout.connect(self.get_scan_data, QtCore.Qt.QueuedConnection)
        
        # For updating scan on the GUI.
        self.sigScanFinishLine.connect(self.get_scan_data, QtCore.Qt.QueuedConnection)
        return

    def on_deactivate(self):
        """ Deactivate properly the confocal scanner dummy.
        """
        self.reset()
        # free memory
        self._scan_image = None
        try:
            self.__update_timer.stop()
        except:
            pass
        self.__update_timer.timeout.disconnect()

    # =========================================================================
    #                         Helper functions
    # =========================================================================
    
    def _convert_dict_mm_to_m(self, param_dict=None):
        """ Convert dict in mm to m, due to the ESP300 being by default in mm.
        """
        return {key: round(value*1e-3, 9) for key, value in param_dict.items()}

    def _convert_dict_m_to_mm(self, param_dict=None):
        """ Convert dict in m to mm, due to the ESP300 being by default in mm.
        """
        return {key: round(value*1e3, 9) for key, value in param_dict.items()}

    def _set_up_hardware_for_scan(self):
        """ Setting up the counter and scanner hardware for scanning. """
        # Turn motors on and set velocity and backlash parameters.
        # self._scanner_hardware.set_up_scanner()

        # Set the triggers and binning for proper count detection on the counting device.
        self._counter_hardware.set_up_counter()
            
    @property
    def scan_settings(self):
        settings = {'axes': tuple(self._current_scan_axes),
                    'range': tuple(self._current_scan_ranges),
                    'resolution': tuple(self._current_scan_resolution),
                    'frequency': self._current_scan_frequency}
        return settings

    # =========================================================================
    #                     ScanningProbeInterface functions
    # =========================================================================

    def reset(self):
        """ Resets the hardware, so the connection is lost and other programs can access it.

        @return int: error code (0:OK, -1:error)
        """
        if -1 in [self._counter_hardware.close_counter(), 
                  self._counter_hardware.close_clock()]:
            return -1
        else:
            return 0

    def get_constraints(self):
        """ Get hardware constraints/limitations.

        @return dict: scanner constraints
        """
        return self._constraints

    def configure_scan(self, scan_settings):
        """ Check whether the given parameters are valid.
        
        @param dict scan_settings:
        @return dict: ALL actually set scan settings.
        """        
        # Sanity checking.
        if self.module_state() != 'idle':
            self.log.error('Unable to configure scan parameters while scan is running. '
                           'Stop scanning and try again.')
            return True, self.scan_settings

        # Get axes, ranges, resolution and frequency from scan_settings.
        axes = scan_settings.get('axes', self._current_scan_axes)
        ranges = tuple((min(r), max(r)) for r in scan_settings.get('range', self._current_scan_ranges))
        resolution = scan_settings.get('resolution', self._current_scan_resolution)
        frequency = float(scan_settings.get('frequency', self._current_scan_frequency))

        # Check if the names of the axes are the same as in _position_ranges.
        if 'axes' in scan_settings:
            if not set(axes).issubset(self._position_ranges):
                self.log.error('Unknown axes names encountered. Valid axes are: {0}'
                               ''.format(set(self._position_ranges)))
                return True, self.scan_settings

        # Check that the dimensions of the parameters are consistent.
        if len(axes) != len(ranges) or len(axes) != len(resolution):
            self.log.error('"axes", "range" and "resolution" must have same length.')
            return True, self.scan_settings
        
        # Check that the parameters are within the constraints.
        for i, ax in enumerate(axes):
            for axis_constr in self._constraints.axes.values():
                if ax == axis_constr.name:
                    break
            if ranges[i][0] < axis_constr.min_value or ranges[i][1] > axis_constr.max_value:
                self.log.error('Scan range out of bounds for axis "{0}". Maximum possible range'
                               ' is: {1}'.format(ax, axis_constr.value_range))
                return True, self.scan_settings
            if resolution[i] < axis_constr.min_resolution or resolution[i] > axis_constr.max_resolution:
                self.log.error('Scan resolution out of bounds for axis "{0}". Maximum possible '
                               'range is: {1}'.format(ax, axis_constr.resolution_range))
                return True, self.scan_settings
            if i == 0:
                if frequency < axis_constr.min_frequency or frequency > axis_constr.max_frequency:
                    self.log.error('Scan frequency out of bounds for fast axis "{0}". Maximum '
                                   'possible range is: {1}'
                                   ''.format(ax, axis_constr.frequency_range))
                    return True, self.scan_settings

        # Then parameters are valid.
        self._current_scan_resolution = tuple(resolution)
        self._current_scan_ranges = ranges
        self._current_scan_axes = tuple(axes)
        self._current_scan_frequency = frequency
        return False, self.scan_settings

    def get_target(self):
        """ Get the current target position of the scanner hardware.

        @return dict: current target position per axis.
        """
        # Get the position from harware in mm.
        target_position_mm = self._scanner_hardware.get_pos()
        
        # Convert dict to m and return position in m.
        target_position_m = self._convert_dict_mm_to_m(target_position_mm)
        return target_position_m

    def get_position(self):
        """ Get a snapshot of the actual scanner position (i.e. from position feedback sensors).

        @return dict: current target position per axis.
        """
        # Get the position from harware in mm.
        position_mm = self._scanner_hardware.get_pos()
        
        # Convert dict to m and return position in m.
        position_m = self._convert_dict_mm_to_m(position_mm)
        return position_m

    def move_absolute(self, position, velocity=None, blocking=False):
        """ Move the scanning probe to an absolute position as fast as 
        possible or with a defined velocity.

        Log error and return current target position if something fails or 
        a 1D/2D scan is in progress.
        """
        # if self.module_state() != 'idle':
        #     self.log.error('Scanning in progress. Unable to move to position.')
        #     return self._current_position.update(self.get_position())
        
        
        if not set(position).issubset(self._position_ranges):
            self.log.error('Invalid axes encountered in position dict. ' 
                           'Valid axes are: {0}'.format(set(self._position_ranges)))
            return self._current_position.update(self.get_position())
            
        else:
            position_mm = self._convert_dict_m_to_mm(position)
            self._scanner_hardware.move_abs(position_mm)
            self._scanner_hardware.wait_for_motion_done()
            
            # self._current_position.update(position)
            self._current_position.update(self.get_position())

        return self._current_position

    def move_relative(self, distance, velocity=None, blocking=False):
        """ Move the scanning probe by a relative distance from the current 
        target position as fast as possible or with a defined velocity.

        Log error and return current target position if something fails or 
        a 1D/2D scan is in progress.
        """
        # if self.module_state() != 'idle':
        #     self.log.error('Scanning in progress. Unable to move relative.')
        #     return self._current_position.update(self.get_position())
        
        
        if not set(distance).issubset(self._position_ranges):
            self.log.error('Invalid axes encountered in distance dict. '
                           'Valid axes are: {0}'.format(set(self._position_ranges)))
            return self._current_position.update(self.get_position())
        
        else:
            distance_mm = self._convert_dict_m_to_mm(distance)
            self._scanner_hardware.move_rel(distance_mm)
            self._scanner_hardware.wait_for_motion_done()
            
            # new_pos = {key: value + self._current_position[key] for key, value in distance.items()}
            # self._current_position.update(new_pos)
            self._current_position.update(self.get_position())

        return self._current_position
    
    def start_scan(self):
        """ Perform the scan routine. 
        
        @return int: error code (0:OK, -1:error)
        """
        # Configure scan
        self.configure_scan(self.scan_settings)
        
        # Check for module state
        if self.module_state() != 'idle':
            self.log.error('Can not start scan. Scan already in progress.')
            return -1
        self.module_state.lock()
    
        # Save current position (marker position)
        self._x_pos_before_scan = self.get_position()['x']
        self._y_pos_before_scan = self.get_position()['y']
    
        # Get the scan starting and ending positions
        self.x_start_pos = round(self._x_pos_before_scan + self._current_scan_ranges[0][0], 9)
        self.y_start_pos = round(self._y_pos_before_scan + self._current_scan_ranges[1][0], 9)
        self.x_end_pos = round(self._x_pos_before_scan + self._current_scan_ranges[0][1], 9)
        self.y_end_pos = round(self._y_pos_before_scan + self._current_scan_ranges[1][1], 9)
        
        # Generate the grid
        self.x_values = np.linspace(self.x_start_pos, self.x_end_pos, self._current_scan_resolution[0])
        if len(self._current_scan_axes) == 2:
            self.y_values = np.linspace(self.y_start_pos, self.y_end_pos, self._current_scan_resolution[1])
        else:
            self.y_values = np.linspace(self._current_position['y'], self._current_position['y'], 1)
        
        # Calculate step sizes
        x_full_range = self._current_scan_ranges[0][1] - self._current_scan_ranges[0][0]
        y_full_range = self._current_scan_ranges[1][1] - self._current_scan_ranges[1][0]
        self.x_step = round(x_full_range / self._current_scan_resolution[0], 9)
        self.y_step = round(y_full_range / self._current_scan_resolution[1], 9)
    
        # Check minimum step
        _current_min_step = min(self.x_step, self.y_step)
        if _current_min_step < 50e-9:  # TODO: use _position_accuracy instead?
            self.log.warning(f'Steps ({_current_min_step} m) too small for hardware to handle '
                             '(min: 50 nm). Please fix scan resolution or position ranges.')
    
        # Initialize scan data
        self._initialize_scan_data()
    
        # Timer
        self.__scan_start = time.time()
    
        # Move to starting position
        self._move_to_start_position()
    
        # Perform scan
        self._perform_scan()
    
        # Return to initial position
        self._move_to_initial_position()
    
        return 0
    
    # =========================================================================
    #                     Helpers for start_scan function:
    # =========================================================================
    def _initialize_scan_data(self):
        """ Initialize scan data. """
        if self._constraints.has_position_feedback:
            feedback_axes = tuple(self._constraints.axes.values())
        else:
            feedback_axes = None
    
        self._scan_data = ScanData(
            channels=tuple(self._constraints.channels.values()),
            scan_axes=tuple(self._constraints.axes[ax] for ax in self._current_scan_axes),
            scan_range=self._current_scan_ranges,
            scan_resolution=self._current_scan_resolution,
            scan_frequency=self._current_scan_frequency,
            position_feedback_axes=feedback_axes,
            target_at_start=self.get_target())
    
        self._scan_data.new_scan()
        
        self.last_scanned_line = np.zeros(self._current_scan_resolution[0])
    
    def _move_to_start_position(self):
        """ Move to the starting position of the scan. """
        self.move_relative({'x': self._current_scan_ranges[0][0]})
        self.move_relative({'y': self._current_scan_ranges[1][0]})
                                    
    def _perform_scan(self):
        """ Perform the scan. """
        self._stop_requested = False
        for line, value_y in enumerate(self.y_values):
            self.move_absolute({'y':value_y})
            if not self._stop_requested:
                for j, value_x in enumerate(self.x_values):
                    self.move_absolute({'x':value_x})
                    self.last_scanned_line[j] = self._counter_hardware._get_counter_hist()

                self._store_scan_data(line)
                self.move_absolute({'x':self.x_start_pos})

    def _store_scan_data(self, line):
        """ Store scan data. 
        
        @params int line: the line count.
        """    
        for ch in self._constraints.channels:
            self._scan_data.data[ch][:, line] = self.last_scanned_line.T
    
        self.sigScanFinishLine.emit()
    
    def _move_to_initial_position(self):
        """ Move to the initial position before the scan. """
        self.move_absolute({'x': self._x_pos_before_scan})
        self.move_absolute({'y': self._y_pos_before_scan})

    # =========================================================================

    def stop_scan(self):
        """ Closes the scanner and cleans up afterwards.

        @return int: error code (0:OK, -1:error)
        """
        self.log.warning('Scanning probe interfuse "stop_scan" called.')
        if self.module_state() == 'locked':
            self._stop_requested = True
            self.module_state.unlock()
        return 0

    def emergency_stop(self):
        """
        @return int: error code (0:OK, -1:error)
        """
        try:
            self.module_state.unlock()
        except FysomError:
            pass
        self._scan_image = None
        self.log.warning('Scanner has been emergency stopped.')
        return 0

    def get_scan_data(self):
        """ Simply return the ScanData object. 
        
        @return ScanData: ScanData instance used in the scan.
        """
        # self.log.warning('get_scan_data called.')
        return self._scan_data
    
    # =========================================================================
    #                Helpers get_scan_data function (Timer)
    # =========================================================================

    def __start_timer(self):
        if self.thread() is not QtCore.QThread.currentThread():
            QtCore.QMetaObject.invokeMethod(self.__update_timer,
                                            'start',
                                            QtCore.Qt.BlockingQueuedConnection)
        else:
            self.__update_timer.start()

    def __stop_timer(self):
        if self.thread() is not QtCore.QThread.currentThread():
            QtCore.QMetaObject.invokeMethod(self.__update_timer,
                                            'stop',
                                            QtCore.Qt.BlockingQueuedConnection)
        else:
            self.__update_timer.stop()

    '''
    def get_scan_data(self):
        """
        @return ScanData: ScanData instance used in the scan
        """
        if self._scan_data is None:
            print('nope, no scan data in hardware')
            return None

        if self.module_state() != 'idle':
            elapsed = time.time() - self.__scan_start
            line_time = (self._current_scan_resolution[0] / self._current_scan_frequency)*100


            if self._scan_data.scan_dimension == 2:
                acquired_lines = min(int(np.floor(elapsed / line_time)),
                                      self._current_scan_resolution[1])

                self.log.warning(f'start: {self.__last_line} ___ {acquired_lines}')

                if acquired_lines > 0:
                    if self.__last_line < acquired_lines - 1:
                        if self.__last_line < 0:
                            self.__last_line = 0

                        for ch in self._constraints.channels:
                            self.log.warning(f'data writing: {self.__last_line} ___ {acquired_lines}')
                            tmp = self._scan_image[:, self.__last_line:acquired_lines]
                            self._scan_data.data[ch][:, self.__last_line:acquired_lines] = tmp

                        self.__last_line = acquired_lines - 1
                    if acquired_lines >= self._current_scan_resolution[1]:
                        self.module_state.unlock()
                    elif self.thread() is QtCore.QThread.currentThread():
                        self.__start_timer()


            else:
                acquired_lines = min(int(np.floor(elapsed / line_time)),
                                      self._current_scan_resolution[0])

                if acquired_lines > 0:
                    if self.__last_line < 0:
                        self.__last_line = 0
                    if self.__last_line < acquired_lines - 1:
                        if self.__last_line < 0:
                            self.__last_line = 0

                        for ch in self._constraints.channels:
                            tmp = self._scan_image[self.__last_line:acquired_lines]
                            self._scan_data.data[ch][self.__last_line:acquired_lines] = tmp

                        self.__last_line = acquired_lines - 1
                    if acquired_lines >= self._current_scan_resolution[0]:
                        self.module_state.unlock()
                    elif self.thread() is QtCore.QThread.currentThread():
                        self.__start_timer()


        return self._scan_data
    '''
    # def _perform_scan(self):
    #     """ Perform the scan. """
    #     self._stop_requested = False
    #     for line in range(self._current_scan_resolution[1]):
    #         if not self._stop_requested:
    #             for j in range(self._current_scan_resolution[0]):
    #                 self.last_scanned_line[j] = self._counter_hardware._get_counter_hist()
    #                 self.move_relative({'x': self.x_step})
                        
    #         self.move_relative({'x': -(self._current_scan_resolution[0]*self.x_step)})
    #         self._store_scan_data(line)
    #         self.move_relative({'y': self.y_step})