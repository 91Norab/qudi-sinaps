# -*- coding: utf-8 -*-
"""
This file contains the Qudi hardware module for the MultiHarp150.

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

import ctypes as ct
# import numpy as np
# import time
# import os
# from qtpy import QtCore

from qudi.core.configoption import ConfigOption
# from qudi.util.paths import get_main_dir
from qudi.util.mutex import Mutex
from qudi.interface.pulser_interface import PulserInterface

error_code = {0: 'PDLM_ERROR_NONE',
              -1: 'PDLM_ERROR_DEVICE_NOT_FOUND',
              -2: 'PDLM_ERROR_NOT_CONNECTED',
              -3: 'PDLM_ERROR_ALREADY_CONNECTED',
              -4: 'PDLM_ERROR_WRONG_USBIDX',
              -5: 'PDLM_ERROR_ILLEGAL_INDEX',
              -6: 'PDLM_ERROR_ILLEGAL_VALUE',
              -7: 'PDLM_ERROR_USB_MSG_INTEGRITY_VIOLATED',
              -8: 'PDLM_ERROR_ILLEGAL_NODEINDEX',
              -9: 'PDLM_ERROR_WRONG_PARAMETER',
              -10: 'PDLM_ERROR_INCOMPATIBLE_FW',
              -11: 'PDLM_ERROR_WRONG_SERIALNUMBER',
              -12: 'PDLM_ERROR_WRONG_PRODUCTMODEL',
              -13: 'PDLM_ERROR_BUFFER_TOO_SMALL',
              -14: 'PDLM_ERROR_INDEX_NOT_FOUND',
              -15: 'PDLM_ERROR_FW_MEMORY_ALLOCATION_ERROR',
              -16: 'PDLM_ERROR_FREQUENCY_TOO_HIGH',
              -17: 'PDLM_ERROR_DEVICE_BUSY_OR_BLOCKED',
              -18: 'PDLM_ERROR_USB_INAPPROPRIATE_DEVICE',
              -19: 'PDLM_ERROR_USB_GET_DSCR_FAILED',
              -20: 'PDLM_ERROR_USB_INVALID_HANDLE',
              -21: 'PDLM_ERROR_USB_INVALID_DSCRBUF',
              -22: 'PDLM_ERROR_USB_IOCTL_FAILED',
              -23: 'PDLM_ERROR_USB_VCMD_FAILED',
              -24: 'PDLM_ERROR_USB_NO_SUCH_PIPE',
              -25: 'PDLM_ERROR_USB_REGNTFY_FAILED',
              -26: 'PDLM_ERROR_USBDRIVER_NO_MEMORY',
              -27: 'PDLM_ERROR_DEVICE_ALREADY_OPENED',
              -28: 'PDLM_ERROR_OPEN_DEVICE_FAILED',
              -29: 'PDLM_ERROR_USB_UNKNOWN_DEVICE',
              -30: 'PDLM_ERROR_EMPTY_QUEUE',
              -31: 'PDLM_ERROR_FEATURE_NOT_AVAILABLE',
              -32: 'PDLM_ERROR_UNINITIALIZED_DATA',
              -33: 'PDLM_ERROR_DLL_MEMORY_ALLOCATION_ERROR',
              -34: 'PDLM_ERROR_UNKNOWN_TAG',
              -35: 'PDLM_ERROR_OPEN_FILE',
              -36: 'PDLM_ERROR_FW_FOOTER',
              -37: 'PDLM_ERROR_FIRMWARE_UPDATE',
              -38: 'PDLM_ERROR_FIRMWARE_UPDATE_RUNNING',
              -39: 'PDLM_ERROR_INCOMPATIBLE_HARDWARE',
              -40: 'PDLM_ERROR_VALUE_NOT_AVAILABLE',
              -41: 'PDLM_ERROR_USB_SET_TIMED_OUT',
              -42: 'PDLM_ERROR_USB_GET_TIMED_OUT',
              -43: 'PDLM_ERROR_USB_SET_FAILED',
              -44: 'PDLM_ERROR_USB_GET_FAILED',
              -45: 'PDLM_ERROR_USB_DATA_SIZE_TOO_BIG',
              -46: 'PDLM_ERROR_FW_VERSION_CHECK',
              -47: 'PDLM_ERROR_WRONG_DRIVER',
              -48: 'PDLM_ERROR_WINUSB_STORED_ERROR',
              -999: 'PDLM_ERROR_UNKNOWN_ERRORCODE',
              -1000: 'PDLM_ERROR_HW_ERROR_OFFSET',
              -2999: 'PDLM_ERROR_HW_MAX_ERROR_NUM',
              -9000: 'PDLM_ERROR_FUNCTION_IS_PQ_INTERNAL',
              -9999: 'PDLM_ERROR_FUNCTION_NOT_IMPLEMENTED_YET'}


class TLaserData(ct.Structure):
    _fields_ = []

class TLaserInfo(ct.Structure):
    pdlm_ldh_string_length = 16
    _fields_ = [("LType", ct.c_char * pdlm_ldh_string_length),
                ("Date", ct.c_char * pdlm_ldh_string_length),
                ("LClass", ct.c_char * pdlm_ldh_string_length)]

class TaikoPDLM1(PulserInterface):
    """
    """

    # Config option
    _usb_index = ConfigOption('usb_id', 0, missing='warn')  # a USB index from 0 to 7.
    _dll_name = ConfigOption('dll', 'PDLM_Lib')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._dll = ct.cdll.LoadLibrary(self._dll_name)
        # C:\ProgramData\Taiko PDL M1\API\Demos\Python
        self._set_constants()
        self.connected_to_device = False
        self.threadlock = Mutex()

    def on_activate(self):
        pass

    def on_deactivate(self):
        pass

    def _set_constants(self):
        self._buffer_len = 1024
        self.PDLM_LIBVERSION_MAXLEN = 12

        self.PDLM_USB_INDEX_MIN = 0
        self.PDLM_USB_INDEX_MAX = 7
        self.PDLM_MAX_USBDEVICES = 8

        self.PDLM_HW_ERRORSTRING_MAXLEN = 47
        self.PDLM_HW_INFO_MAXLEN = 36
        self.PDLM_DEV_STRING_LENGTH = 16
        self.pdlm_ldh_string_length = 16

    def check(self, func_val):
        """ Check routine for the received error codes.

        @param int func_val: return error code of the called function.
        @return int: pass the error code further so that other functions have the possibility to use it.
        """
        if not func_val == 0:
            self.log.error(f'Error in Taiko PDL M1 with errorcode {func_val}: {error_code[func_val]}')
        return func_val

    # =========================================================================
    #                           Hardware functions
    # =========================================================================

    # =========================================================================
    # Interface Functions.
    # =========================================================================

    def _get_version(self):
        """ Get the version number of the currently installed library.

        @return string: the version number of the current library.
        """
        lib_version_buffer = ct.create_string_buffer(self._buffer_len)
        self.check(self._dll.PDLM_GetLibraryVersion(ct.byref(lib_version_buffer), self._buffer_len))
        return lib_version_buffer.value.decode()  # .decode() converts byte to string

    def _is_running_in_wine(self):
        """ Return the boolean information whether the library is running in a Wine environment,
        which may be relevant for support cases. Besides this, this function is solely informative.

        @return bool: true if running in a Wine environment on a POSIX system.
        """
        wine_buffer = ct.c_uint32()
        self.check(self._dll.PDLM_LibIsRunningInWine(ct.byref(wine_buffer)))
        return wine_buffer.value

    def _get_usb_driver_info(self):
        """ Provides information on the USB driver (driver service name, driver version, and driver date).

        @return tuple(3): (driver service name, driver version, driver date).
        """
        driver_name_buffer = ct.create_string_buffer(self._buffer_len)
        driver_version_buffer = ct.create_string_buffer(self._buffer_len)
        driver_date_buffer = ct.create_string_buffer(self._buffer_len)
        self.check(self._dll.PDLM_GetUSBDriverInfo(ct.byref(ct.byref(driver_name_buffer), self._buffer_len,
                                                            ct.byref(driver_version_buffer), self._buffer_len,
                                                            ct.byref(driver_date_buffer), self._buffer_len)))
        return (driver_name_buffer.value.decode(), driver_version_buffer.value.decode(),
                driver_date_buffer.value.decode())

    def _decode_error(self, err_code):
        """ Get the string error code from the device.

        @param int err_code: the error number from 0 and below.
        @return str: string error code.
        """
        err_string_buffer = ct.create_string_buffer(self._buffer_len)
        buf_len = ct.c_uint(self._buffer_len - 1)
        self.check(self._dll.PDLM_DecodeError(err_code, ct.byref(err_string_buffer), ct.byref(buf_len)))
        return err_string_buffer.value.decode()

    def _get_tag_description(self, tag):
        """ Get the description of the specified tag.

        @param int tag: the type code of the tag.
        @return tuple(2): (tag_type_code, tag_name).
            int tag_type_code: pointer to an unsigned integer variable that returns the type code of the tag.
            str tag_name: pointer to a string variable that returns the name of the tag.
        """
        tag_type_code = ct.c_uint32()
        tag_name = ct.create_string_buffer(32)
        self.check(self._dll.PDLM_GetTagDescription(tag, ct.byref(tag_type_code), ct.byref(tag_name)))
        return tag_type_code.value, tag_name.value.decode()

    def _decode_pulse_shape(self, shape):
        """ Provides a human-readable description of a pulse shape code.
        These codes indicate in which range the laser head is currently operating.

        @param int shape: the tag code, valid values are:
                        0: Broadened pulse regime (due to high power settings)
                        1: Narrow pulse regime or “single pulse”
                        (laser diode is operating at a pulse width corresponding to its specification)
                        2: Sub-threshold (or “LED domain”). No lasing occurs, only spontaneous emission
                        3: Unknown pulse shape
        @return str: description of the pulse shape.
        """
        buffer = ct.create_string_buffer(self._buffer_len)
        buf_len = ct.c_uint(self._buffer_len - 1)
        self.check(self._dll.PDLM_DecodePulseShape(shape, ct.byref(buffer), ct.byref(buf_len)))
        return buffer.value.decode()

    def _decode_lh_features(self, lh_features):
        """ Turns the bit encoded feature list of a laser head into a human-readable list with each field separated
        by a semi-colon (;).

        @param int lh_features: bit encoded laser head feature list.
        @return str: description of the laser head feature list.
        """
        buffer = ct.create_string_buffer(self._buffer_len)
        buf_len = ct.c_uint(self._buffer_len - 1)
        self.check(self._dll.PDLM_DecodeLHFeatures(lh_features, ct.byref(buffer), ct.byref(buf_len)))
        return buffer.value.decode()

    def _decode_system_status(self, state):
        """ Decodes the status code to a human-readable string.
        Note that texts corresponding to each of the status bits set are separated by a semi-colon (;)

        @param int state: status code.
        @return str: description of the system status.
        """
        buffer = ct.create_string_buffer(self._buffer_len)
        buf_len = ct.c_uint(self._buffer_len - 1)
        self.check(self._dll.PDLM_DecodeSystemStatus(state, ct.byref(buffer), ct.byref(buf_len)))
        return buffer.value.decode()

    # =========================================================================
    # Basic Device Functions.
    # =========================================================================

    def _open_connection(self):
        """ Open a connection to this device. """
        serial_num = ct.create_string_buffer(8)
        ret = self.check(self._dll.PDLM_OpenDevice(self._usb_index, ct.byref(serial_num)))
        if ret >= 0:
            self.connected_to_device = True
            self.log.info('Connection to the Taiko PDL M1 established')

    def _close_connection(self):
        """ Close the connection to the device. """
        self.connected_to_device = False
        self.check(self._dll.PDLM_CloseDevice(self._usb_index))
        self.log.info('Connection to the Taiko PDL M1 closed.')

    def _open_get_ser_num_and_close(self):
        """ Non-exclusively opens the device associated with the given USB index.

        @return str: serial number (even for blocked devices).
        """
        serial_num = ct.create_string_buffer(8)
        self.check(self._dll.PDLM_OpenGetSerNumAndClose(self._usb_index, ct.byref(serial_num)))
        return serial_num.value.decode()

    def set_exclusive_ui(self, mode):
        """ Set the UI access mode.

        @param int mode: the desired UI access mode (see manual for valid values).
        """
        self.check(self._dll.PDLM_SetExclusiveUI(self._usb_index, mode))

    def _get_exclusive_ui(self):
        """ Read out the current UI access mode state.

        @return int: the current UI access mode state.
        """
        mode = ct.c_uint32()
        self.check(self._dll.PDLM_GetExclusiveUI(self._usb_index, ct.byref(mode)))
        return mode.value

    # =========================================================================
    # Device Information Functions.
    # =========================================================================

    def _get_usb_str_descriptor(self):
        """ Get the USB string descriptors for the device associated to the USB index, separated by a semi-colon (;).

        @return str: string with concatenated USB string descriptors.
        """
        buffer_usb_descriptor = ct.create_string_buffer(self._buffer_len)
        buf_len = ct.c_uint(self._buffer_len)
        self.check(self._dll.PDLM_GetUSBStrDescriptor(self._usb_index, ct.byref(buffer_usb_descriptor),
                                                      ct.byref(buf_len)))
        return buffer_usb_descriptor.value.decode()

    def _get_hardware_info(self):
        """ Get the hardware info, as usually shown in the About box/Support info texts.
        This information mainly identifies the hardware product type and version.

        @return str: string containing the hardware info.
        """
        buffer_info = ct.create_string_buffer(self._buffer_len)
        buf_len = ct.c_uint(self._buffer_len)
        self.check(self._dll.PDLM_GetHardwareInfo(self._usb_index, ct.byref(buffer_info), ct.byref(buf_len)))
        return buffer_info.value.decode()

    def _create_support_request_text(self, option=None):  # To test: can 'option' be optional (=None)?
        """ Generate common hardware, software, and environment information, as can be usually found in the “About...”
        or the support information boxes. This sting contains all relevant information about the device associated to
        the given USB index (including version numbers, environment, feature and option lists).

        @param int option: The output string can be customized via the "option" bitset:
            Bit Value: 0x01 -> Effect: if included, the preamble will be suppressed.
            Bit Value: 0x02 -> Effect: if included, the title will be suppressed.
            Bit Value: 0x04 -> Effect: if included, the info on calling software will not be indented.
            Bit Value: 0x08 -> Effect: if included, the system info will be suppressed.
        @return str: string containing common hardware, software, and environment information.
        """
        buffer_preamble = ct.create_string_buffer(self._buffer_len)
        buffer_calling_sw = ct.create_string_buffer(self._buffer_len)
        buffer = ct.create_string_buffer(self._buffer_len)
        buf_len = ct.c_uint(self._buffer_len)
        self.check(self._dll.PDLM_CreateSupportRequestText(self._usb_index, ct.byref(buffer_preamble),
                                                           ct.byref(buffer_calling_sw), option,
                                                           ct.byref(buf_len), ct.byref(buffer)))
        return buffer.value.decode()

    def _get_fw_version(self):
        """ Read out the firmware version of the device associated with the given USB index.

        @return str: the firmware version.
        """
        buffer_fw_version = ct.create_string_buffer(self._buffer_len)
        buf_len = ct.c_uint(self._buffer_len)
        self.check(self._dll.PDLM_GetFWVersion(self._usb_index, ct.byref(buffer_fw_version), ct.byref(buf_len)))
        return buffer_fw_version.value.decode()

    def _get_fpga_version(self):
        """ Reads out the FPGA version of the device associated with the given USB index.

        @return str: the FPGA version.
        """
        buffer_fpga_version = ct.create_string_buffer(self._buffer_len)
        buf_len = ct.c_uint(self._buffer_len)
        self.check(self._dll.PDLM_GetFPGAVersion(self._usb_index, ct.byref(buffer_fpga_version), ct.byref(buf_len)))
        return buffer_fpga_version.value.decode()

    # PDLM_GetDeviceData
    def _get_device_data(self):
        pass

    # =========================================================================
    # Laser Head Information Functions.
    # =========================================================================

    def _get_lh_version(self):
        """ Get the version number of the laser head connected to the device associated with the given USB index.

        @return str: the version number of the laser head.
        """
        buffer_lh_version = ct.create_string_buffer(self._buffer_len)
        buf_len = ct.c_uint(self._buffer_len)
        self.check(self._dll.PDLM_GetLHVersion(self._usb_index, ct.byref(buffer_lh_version), ct.byref(buf_len)))
        return buffer_lh_version.value.decode()

    def _get_lh_data(self):
        pass
        # buffer_lh_data = TLaserData()
        # size = ct.sizeof(lh_info)
        # self.check(self._dll.PDLM_GetLHData(self._usb_index, ct.byref(buffer_lh_data), ct.byref(size)))
        # return buffer_lh_data.value.decode()


    def _get_lh_info(self):
        """ Get additional information about the laser head connected to the device associated with the given USB index.

        @return tuple(3): (LType, Date, LClass):
            LType: Designation of laser head (e.g., "LDH-IX-B-405").
            Date: Date of manufacturing (format: yyyy-mm-dd).
            LClass: Laser Class that is applicable to this head (e.g., "3R").
        """
        buffer_lh_info = TLaserInfo()
        size = ct.sizeof(buffer_lh_info)
        self.check(self._dll.PDLM_GetLHInfo(self._usb_index, ct.byref(buffer_lh_info), ct.byref(size)))
        return buffer_lh_info.value.decode()

    def _get_lh_features(self):
        """ Get all laser head features.

        @return int: a bit code, see manual for the table of laser head feature bits.
        """
        lh_features = ct.c_uint32()
        self.check(self._dll.PDLM_GetLHFeatures(self._usb_index, ct.byref(lh_features)))
        return lh_features.value

    # =========================================================================
    # Status and Error Information Functions.
    # =========================================================================

    def _set_hwnd(self, hwnd):
        """ TO DO """
        self.check(self._dll.PDLM_SetHWND(self._usb_index, hwnd))

    def _get_system_status(self):
        """ Get the state code (bit-coded). Refer to the API manual for the table of all assigned status bits
        and for the list of useful status bit masks. """
        mode = ct.c_uint32()
        self.check(self._dll.PDLM_GetSystemStatus(self._usb_index, ct.byref(mode)))
        return mode.value

    # PDLM_GetQueuedChanges
    def _get_queued_changes(self):
        pass

    # PDLM_GetTagValueList
    def _get_tag_value_list(self):
        pass

    # PDLM_GetQueuedError
    def _get_queued_error(self):
        pass

    # PDLM_GetQueuedErrorString
    def _get_queued_error_string(self):
        pass

    # =========================================================================
    # Laser Locking Functions.
    # =========================================================================

    # PDLM_GetLocked
    def _get_locked(self):
        pass

    # PDLM_SetSoftLock
    def _set_soft_lock(self):
        pass

    # PDLM_GetSoftLock
    def _get_soft_lock(self):
        pass

    # =========================================================================
    # Laser Emission Mode Functions.
    # =========================================================================

    # PDLM_SetLaserMode
    def _set_laser_mode(self):
        pass

    # PDLM_GetLaserMode
    def _get_laser_mode(self):
        pass

    # PDLM_SetLDHPulsePowerTable
    def _set_ldh_pulse_power_table(self):
        pass

    # PDLM_GetLDHPulsePowerTable
    def _get_ldh_pulse_power_table(self):
        pass

    # =========================================================================
    # Triggering and Gating Functions.
    # =========================================================================

    # PDLM_SetTriggerMode
    def _set_trigger_mode(self):
        pass

    # PDLM_GetTriggerMode
    def _get_trigger_mode(self):
        pass

    # PDLM_GetTriggerLevelLimits
    def _get_trigger_level_limits(self):
        pass

    # PDLM_SetTriggerLevel
    def _set_trigger_level(self):
        pass

    # PDLM_GetTriggerLevel
    def _get_trigger_level(self):
        pass

    # PDLM_GetExtTriggerFrequency
    def _get_ext_trigger_frequency(self):
        pass

    # PDLM_SetFastGate
    def _set_fast_gate(self):
        pass

    # PDLM_GetFastGate
    def _get_fast_gate(self):
        pass

    # PDLM_SetFastGateImp
    def _set_fast_gate_imp(self):
        pass

    # PDLM_GetFastGateImp
    def _get_fast_gate_imp(self):
        pass

    # PDLM_SetSlowGate
    def _set_slow_gate(self):
        pass

    # PDLM_GetSlowGate
    def _get_slow_gate(self):
        pass

    # =========================================================================
    # Pulse Frequency and Burst Setting Functions.
    # =========================================================================

    # PDLM_GetFrequencyLimits
    def _get_frequency_limits(self):
        pass

    # PDLM_SetFrequency
    def _set_frequency(self):
        pass

    # PDLM_GetFrequency
    def _get_frequency(self):
        pass

    # PDLM_SetBurst
    def _set_burst(self):
        pass

    # PDLM_GetBurst
    def _get_burst(self):
        pass

    # =========================================================================
    # Functions for Temperature Settings.
    # =========================================================================

    # PDLM_SetTempScale
    def _set_temp_scale(self):
        pass

    # PDLM_GetTempScale
    def _get_temp_scale(self):
        pass

    # PDLM_GetLHTargetTempLimits
    def _get_lh_target_temp_limits(self):
        pass

    # PDLM_SetLHTargetTemp
    def _set_lh_target_temp(self):
        pass

    # PDLM_GetLHTargetTemp
    def _get_lh_target_temp(self):
        pass

    # PDLM_GetLHCurrentTemp
    def _get_lh_current_temp(self):
        pass

    # PDLM_GetLHCaseTemp
    def _get_lh_case_temp(self):
        pass

    # PDLM_GetLHWavelength
    def _get_lh_wavelength(self):
        pass

    # =========================================================================
    # Laser Head Functions for Pulse Power Settings.
    # =========================================================================

    # PDLM_GetPulsePowerLimits
    def _get_pulse_power_limits(self):
        pass

    # PDLM_SetPulsePower
    def _set_pulse_power(self):
        pass

    # PDLM_GetPulsePower
    def _get_pulse_power(self):
        pass

    # PDLM_SetPulsePowerPermille
    def _set_pulse_power_permille(self):
        pass

    # PDLM_GetPulsePowerPermille
    def _get_pulse_power_permille(self):
        pass

    # PDLM_SetPulsePowerNanowatt
    def _set_pulse_power_nanowatt(self):
        pass

    # PDLM_GetPulsePowerNanowatt
    def _get_pulse_power_nanowatt(self):
        pass

    # PDLM_GetPulseShape
    def _get_pulse_shape(self):
        pass

    # =========================================================================
    # Laser Head Functions for CW Power Settings.
    # =========================================================================

    # PDLM_GetCwPowerLimits
    def _get_cw_power_limits(self):
        pass

    # PDLM_SetCwPower
    def _set_cw_power(self):
        pass

    # PDLM_GetCwPower
    def _get_cw_power(self):
        pass

    # PDLM_SetCwPowerPermille
    def _set_cw_power_permille(self):
        pass

    # PDLM_GetCwPowerPermille
    def _get_cw_power_permille(self):
        pass

    # PDLM_SetCwPowerMicrowatt
    def _set_cw_power_microwatt(self):
        pass

    # PDLM_GetCwPowerMicrowatt
    def _get_cw_power_microwatt(self):
        pass

    # =========================================================================
    # Special Laser Head Functions.
    # =========================================================================

    # PDLM_SetLHFan
    def _set_lh_fan(self):
        pass

    # PDLM_GetLHFan
    def _get_lh_fan(self):
        pass

    # =========================================================================
    # Preset Functions.
    # =========================================================================

    # PDLM_StorePreset
    def _store_preset(self):
        pass

    # PDLM_GetPresetInfo
    def _get_preset_info(self):
        pass

    # PDLM_GetPresetText
    def _get_preset_text(self):
        pass

    # PDLM_RecallPreset
    def _recall_preset(self):
        pass

    # PDLM_ErasePreset
    def _erase_preset(self):
        pass


    # =========================================================================
    #                        PulserInterface Commands
    # =========================================================================


    def get_constraints(self):
        """
        Retrieve the hardware constrains from the Pulsing device.

        @return constraints object: object with pulser constraints as attributes.

        Provides all the constraints (e.g. sample_rate, amplitude, total_length_bins,
        channel_config, ...) related to the pulse generator hardware to the caller.

            SEE PulserConstraints CLASS IN pulser_interface.py FOR AVAILABLE CONSTRAINTS!!!

        If you are not sure about the meaning, look in other hardware files to get an impression.
        If still additional constraints are needed, then they have to be added to the
        PulserConstraints class.

        Each scalar parameter is an ScalarConstraints object defined in core.util.interfaces.
        Essentially it contains min/max values as well as min step size, default value and unit of
        the parameter.

        PulserConstraints.activation_config differs, since it contain the channel
        configuration/activation information of the form:
            {<descriptor_str>: <channel_set>,
             <descriptor_str>: <channel_set>,
             ...}

        If the constraints cannot be set in the pulsing hardware (e.g. because it might have no
        sequence mode) just leave it out so that the default is used (only zeros).

        # Example for configuration with default values:
        constraints = PulserConstraints()

        constraints.sample_rate.min = 10.0e6
        constraints.sample_rate.max = 12.0e9
        constraints.sample_rate.step = 10.0e6
        constraints.sample_rate.default = 12.0e9

        constraints.a_ch_amplitude.min = 0.02
        constraints.a_ch_amplitude.max = 2.0
        constraints.a_ch_amplitude.step = 0.001
        constraints.a_ch_amplitude.default = 2.0

        constraints.a_ch_offset.min = -1.0
        constraints.a_ch_offset.max = 1.0
        constraints.a_ch_offset.step = 0.001
        constraints.a_ch_offset.default = 0.0

        constraints.d_ch_low.min = -1.0
        constraints.d_ch_low.max = 4.0
        constraints.d_ch_low.step = 0.01
        constraints.d_ch_low.default = 0.0

        constraints.d_ch_high.min = 0.0
        constraints.d_ch_high.max = 5.0
        constraints.d_ch_high.step = 0.01
        constraints.d_ch_high.default = 5.0

        constraints.waveform_length.min = 80
        constraints.waveform_length.max = 64800000
        constraints.waveform_length.step = 1
        constraints.waveform_length.default = 80

        constraints.waveform_num.min = 1
        constraints.waveform_num.max = 32000
        constraints.waveform_num.step = 1
        constraints.waveform_num.default = 1

        constraints.sequence_num.min = 1
        constraints.sequence_num.max = 8000
        constraints.sequence_num.step = 1
        constraints.sequence_num.default = 1

        constraints.subsequence_num.min = 1
        constraints.subsequence_num.max = 4000
        constraints.subsequence_num.step = 1
        constraints.subsequence_num.default = 1

        # If sequencer mode is available then these should be specified
        constraints.repetitions.min = 0
        constraints.repetitions.max = 65539
        constraints.repetitions.step = 1
        constraints.repetitions.default = 0

        constraints.event_triggers = ['A', 'B']
        constraints.flags = ['A', 'B', 'C', 'D']

        constraints.sequence_steps.min = 0
        constraints.sequence_steps.max = 8000
        constraints.sequence_steps.step = 1
        constraints.sequence_steps.default = 0

        # the name a_ch<num> and d_ch<num> are generic names, which describe UNAMBIGUOUSLY the
        # channels. Here all possible channel configurations are stated, where only the generic
        # names should be used. The names for the different configurations can be customary chosen.
        # IMPORTANT: Active channel sets must be of type frozenset to properly work as remote module
        activation_conf = dict()
        activation_conf['yourconf'] = frozenset(
            {'a_ch1', 'd_ch1', 'd_ch2', 'a_ch2', 'd_ch3', 'd_ch4'})
        activation_conf['different_conf'] = frozenset({'a_ch1', 'd_ch1', 'd_ch2'})
        activation_conf['something_else'] = frozenset({'a_ch2', 'd_ch3', 'd_ch4'})
        constraints.activation_config = activation_conf
        """
        pass

    def pulser_on(self):
        """ Switches the pulsing device on.

        @return int: error code (0:OK, -1:error)
        """
        pass

    def pulser_off(self):
        """ Switches the pulsing device off.

        @return int: error code (0:OK, -1:error)
        """
        pass

    def load_waveform(self, load_dict):
        """ Loads a waveform to the specified channel of the pulsing device.

        @param dict|list load_dict: a dictionary with keys being one of the available channel
                                    index and values being the name of the already written
                                    waveform to load into the channel.
                                    Examples:   {1: rabi_ch1, 2: rabi_ch2} or
                                                {1: rabi_ch2, 2: rabi_ch1}
                                    If just a list of waveform names if given, the channel
                                    association will be invoked from the channel
                                    suffix '_ch1', '_ch2' etc.

                                        {1: rabi_ch1, 2: rabi_ch2}
                                    or
                                        {1: rabi_ch2, 2: rabi_ch1}

                                    If just a list of waveform names if given,
                                    the channel association will be invoked from
                                    the channel suffix '_ch1', '_ch2' etc. A
                                    possible configuration can be e.g.

                                        ['rabi_ch1', 'rabi_ch2', 'rabi_ch3']

        @return dict: Dictionary containing the actually loaded waveforms per
                      channel.

        For devices that have a workspace (i.e. AWG) this will load the waveform
        from the device workspace into the channel. For a device without mass
        memory, this will make the waveform/pattern that has been previously
        written with self.write_waveform ready to play.

        Please note that the channel index used here is not to be confused with the number suffix
        in the generic channel descriptors (i.e. 'd_ch1', 'a_ch1'). The channel index used here is
        highly hardware specific and corresponds to a collection of digital and analog channels
        being associated to a SINGLE wavfeorm asset.
        """
        pass

    def load_sequence(self, sequence_name):
        """ Loads a sequence to the channels of the device in order to be ready for playback.
        For devices that have a workspace (i.e. AWG) this will load the sequence from the device
        workspace into the channels.
        For a device without mass memory this will make the waveform/pattern that has been
        previously written with self.write_waveform ready to play.

        @param dict|list sequence_name: a dictionary with keys being one of the available channel
                                        index and values being the name of the already written
                                        waveform to load into the channel.
                                        Examples:   {1: rabi_ch1, 2: rabi_ch2} or
                                                    {1: rabi_ch2, 2: rabi_ch1}
                                        If just a list of waveform names if given, the channel
                                        association will be invoked from the channel
                                        suffix '_ch1', '_ch2' etc.

        @return dict: Dictionary containing the actually loaded waveforms per channel.
        """
        pass

    def get_loaded_assets(self):
        """
        Retrieve the currently loaded asset names for each active channel of the device.
        The returned dictionary will have the channel numbers as keys.
        In case of loaded waveforms the dictionary values will be the waveform names.
        In case of a loaded sequence the values will be the sequence name appended by a suffix
        representing the track loaded to the respective channel (i.e. '<sequence_name>_1').

        @return (dict, str): Dictionary with keys being the channel number and values being the
                             respective asset loaded into the channel,
                             string describing the asset type ('waveform' or 'sequence')
        """
        pass

    def clear_all(self):
        """ Clears all loaded waveforms from the pulse generators RAM/workspace.

        @return int: error code (0:OK, -1:error)
        """
        pass

    def get_status(self):
        """ Retrieves the status of the pulsing hardware

        @return (int, dict): tuple with an integer value of the current status and a corresponding
                             dictionary containing status description for all the possible status
                             variables of the pulse generator hardware.
        """
        pass

    def get_sample_rate(self):
        """ Get the sample rate of the pulse generator hardware

        @return float: The current sample rate of the device (in Hz)

        Do not return a saved sample rate from an attribute, but instead retrieve the current
        sample rate directly from the device.
        """
        pass

    def set_sample_rate(self, sample_rate):
        """ Set the sample rate of the pulse generator hardware.

        @param float sample_rate: The sampling rate to be set (in Hz)

        @return float: the sample rate returned from the device (in Hz).

        Note: After setting the sampling rate of the device, use the actually set return value for
              further processing.
        """
        pass

    def get_analog_level(self, amplitude=None, offset=None):
        """ Retrieve the analog amplitude and offset of the provided channels.

        @param list amplitude: optional, if the amplitude value (in Volt peak to peak, i.e. the
                               full amplitude) of a specific channel is desired.
        @param list offset: optional, if the offset value (in Volt) of a specific channel is
                            desired.

        @return: (dict, dict): tuple of two dicts, with keys being the channel descriptor string
                               (i.e. 'a_ch1') and items being the values for those channels.
                               Amplitude is always denoted in Volt-peak-to-peak and Offset in volts.

        Note: Do not return a saved amplitude and/or offset value but instead retrieve the current
              amplitude and/or offset directly from the device.

        If nothing (or None) is passed then the levels of all channels will be returned. If no
        analog channels are present in the device, return just empty dicts.

        Example of a possible input:
            amplitude = ['a_ch1', 'a_ch4'], offset = None
        to obtain the amplitude of channel 1 and 4 and the offset of all channels
            {'a_ch1': -0.5, 'a_ch4': 2.0} {'a_ch1': 0.0, 'a_ch2': 0.0, 'a_ch3': 1.0, 'a_ch4': 0.0}
        """
        pass

    def set_analog_level(self, amplitude=None, offset=None):
        """ Set amplitude and/or offset value of the provided analog channel(s).

        @param dict amplitude: dictionary, with key being the channel descriptor string
                               (i.e. 'a_ch1', 'a_ch2') and items being the amplitude values
                               (in Volt peak to peak, i.e. the full amplitude) for the desired
                               channel.
        @param dict offset: dictionary, with key being the channel descriptor string
                            (i.e. 'a_ch1', 'a_ch2') and items being the offset values
                            (in absolute volt) for the desired channel.

        @return (dict, dict): tuple of two dicts with the actual set values for amplitude and
                              offset for ALL channels.

        If nothing is passed then the command will return the current amplitudes/offsets.

        Note: After setting the amplitude and/or offset values of the device, use the actual set
              return values for further processing.
        """
        pass

    def get_digital_level(self, low=None, high=None):
        """ Retrieve the digital low and high level of the provided/all channels.

        @param list low: optional, if the low value (in Volt) of a specific channel is desired.
        @param list high: optional, if the high value (in Volt) of a specific channel is desired.

        @return: (dict, dict): tuple of two dicts, with keys being the channel descriptor strings
                               (i.e. 'd_ch1', 'd_ch2') and items being the values for those
                               channels. Both low and high value of a channel is denoted in volts.

        Note: Do not return a saved low and/or high value but instead retrieve
              the current low and/or high value directly from the device.

        If nothing (or None) is passed then the levels of all channels are being returned.
        If no digital channels are present, return just an empty dict.

        Example of a possible input:
            low = ['d_ch1', 'd_ch4']
        to obtain the low voltage values of digital channel 1 an 4. A possible answer might be
            {'d_ch1': -0.5, 'd_ch4': 2.0} {'d_ch1': 1.0, 'd_ch2': 1.0, 'd_ch3': 1.0, 'd_ch4': 4.0}
        Since no high request was performed, the high values for ALL channels are returned (here 4).
        """
        pass

    def set_digital_level(self, low=None, high=None):
        """ Set low and/or high value of the provided digital channel.

        @param dict low: dictionary, with key being the channel descriptor string
                         (i.e. 'd_ch1', 'd_ch2') and items being the low values (in volt) for the
                         desired channel.
        @param dict high: dictionary, with key being the channel descriptor string
                          (i.e. 'd_ch1', 'd_ch2') and items being the high values (in volt) for the
                          desired channel.

        @return (dict, dict): tuple of two dicts where first dict denotes the current low value and
                              the second dict the high value for ALL digital channels.
                              Keys are the channel descriptor strings (i.e. 'd_ch1', 'd_ch2')

        If nothing is passed then the command will return the current voltage levels.

        Note: After setting the high and/or low values of the device, use the actual set return
              values for further processing.
        """
        pass

    def get_active_channels(self, ch=None):
        """ Get the active channels of the pulse generator hardware.

        @param list ch: optional, if specific analog or digital channels are needed to be asked
                        without obtaining all the channels.

        @return dict:  where keys denoting the channel string and items boolean expressions whether
                       channel are active or not.

        Example for an possible input (order is not important):
            ch = ['a_ch2', 'd_ch2', 'a_ch1', 'd_ch5', 'd_ch1']
        then the output might look like
            {'a_ch2': True, 'd_ch2': False, 'a_ch1': False, 'd_ch5': True, 'd_ch1': False}

        If no parameter (or None) is passed to this method all channel states will be returned.
        """
        pass

    def set_active_channels(self, ch=None):
        """
        Set the active/inactive channels for the pulse generator hardware.
        The state of ALL available analog and digital channels will be returned
        (True: active, False: inactive).
        The actually set and returned channel activation must be part of the available
        activation_configs in the constraints.
        You can also activate/deactivate subsets of available channels but the resulting
        activation_config must still be valid according to the constraints.
        If the resulting set of active channels can not be found in the available
        activation_configs, the channel states must remain unchanged.

        @param dict ch: dictionary with keys being the analog or digital string generic names for
                        the channels (i.e. 'd_ch1', 'a_ch2') with items being a boolean value.
                        True: Activate channel, False: Deactivate channel

        @return dict: with the actual set values for ALL active analog and digital channels

        If nothing is passed then the command will simply return the unchanged current state.

        Note: After setting the active channels of the device, use the returned dict for further
              processing.

        Example for possible input:
            ch={'a_ch2': True, 'd_ch1': False, 'd_ch3': True, 'd_ch4': True}
        to activate analog channel 2 digital channel 3 and 4 and to deactivate
        digital channel 1. All other available channels will remain unchanged.
        """
        pass

    def write_waveform(self, name, analog_samples, digital_samples, is_first_chunk, is_last_chunk,
                       total_number_of_samples):
        """
        Write a new waveform or append samples to an already existing waveform on the device memory.
        The flags is_first_chunk and is_last_chunk can be used as indicator if a new waveform should
        be created or if the write process to a waveform should be terminated.

        NOTE: All sample arrays in analog_samples and digital_samples must be of equal length!

        @param str name: the name of the waveform to be created/append to
        @param dict analog_samples: keys are the generic analog channel names (i.e. 'a_ch1') and
                                    values are 1D numpy arrays of type float32 containing the
                                    voltage samples normalized to half Vpp (between -1 and 1).
        @param dict digital_samples: keys are the generic digital channel names (i.e. 'd_ch1') and
                                     values are 1D numpy arrays of type bool containing the marker
                                     states.
        @param bool is_first_chunk: Flag indicating if it is the first chunk to write.
                                    If True this method will create a new empty wavveform.
                                    If False the samples are appended to the existing waveform.
        @param bool is_last_chunk:  Flag indicating if it is the last chunk to write.
                                    Some devices may need to know when to close the appending wfm.
        @param int total_number_of_samples: The number of sample points for the entire waveform
                                            (not only the currently written chunk)

        @return (int, list): Number of samples written (-1 indicates failed process) and list of
                             created waveform names
        """
        pass

    def write_sequence(self, name, sequence_parameters):
        """
        Write a new sequence on the device memory.

        @param str name: the name of the waveform to be created/append to
        @param list sequence_parameters: List containing tuples of length 2. Each tuple represents
                                         a sequence step. The first entry of the tuple is a list of
                                         waveform names (str); one for each channel. The second
                                         tuple element is a SequenceStep instance containing the
                                         sequencing parameters for this step.

        @return: int, number of sequence steps written (-1 indicates failed process)
        """
        pass

    def get_waveform_names(self):
        """ Retrieve the names of all uploaded waveforms on the device.

        @return list: List of all uploaded waveform name strings in the device workspace.
        """
        pass

    def get_sequence_names(self):
        """ Retrieve the names of all uploaded sequence on the device.

        @return list: List of all uploaded sequence name strings in the device workspace.
        """
        pass

    def delete_waveform(self, waveform_name):
        """ Delete the waveform with name "waveform_name" from the device memory.

        @param str waveform_name: The name of the waveform to be deleted
                                  Optionally a list of waveform names can be passed.

        @return list: a list of deleted waveform names.
        """
        pass

    def delete_sequence(self, sequence_name):
        """ Delete the sequence with name "sequence_name" from the device memory.

        @param str sequence_name: The name of the sequence to be deleted
                                  Optionally a list of sequence names can be passed.

        @return list: a list of deleted sequence names.
        """
        pass

    def get_interleave(self):
        """ Check whether Interleave is ON or OFF in AWG.

        @return bool: True: ON, False: OFF

        Will always return False for pulse generator hardware without interleave.
        """
        pass

    def set_interleave(self, state=False):
        """ Turns the interleave of an AWG on or off.

        @param bool state: The state the interleave should be set to
                           (True: ON, False: OFF)

        @return bool: actual interleave status (True: ON, False: OFF)

        Note: After setting the interleave of the device, retrieve the
              interleave again and use that information for further processing.

        Unused for pulse generator hardware other than an AWG.
        """
        pass

    def reset(self):
        """ Reset the device.

        @return int: error code (0:OK, -1:error)
        """
        pass