# -*- coding: utf-8 -*-
"""
Qudi is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Qudi is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Qudi. If not, see <http://www.gnu.org/licenses/>.

Copyright (c) the Qudi Developers. See the COPYRIGHT.txt file at the
top-level directory of this distribution and at <https://github.com/Ulm-IQO/qudi/>
"""

import os
import ctypes as ct
from qudi.core.module import Base
from qudi.core.configoption import ConfigOption
from qudi.util.datastorage import TextDataStorage
from qudi.util.paths import get_default_data_dir

# FTDI and Communication errors
error_codes = {
    0: "FT_OK - Success",
    1: "FT_InvalidHandle - The FTDI functions have not been initialized.",
    2: "FT_DeviceNotFound - The Device could not be found. This can be generated if the function TLI_BuildDeviceList() has not been called.",
    3: "FT_DeviceNotOpened - The Device must be opened before it can be accessed. See the appropriate Open function for your device.",
    4: "FT_IOError - An I/O Error has occured in the FTDI chip.",
    5: "FT_InsufficientResources - There are Insufficient resources to run this application.",
    6: "FT_InvalidParameter - An invalid parameter has been supplied to the device.",
    7: "FT_DeviceNotPresent - The Device is no longer present. The device may have been disconnected since the last TLI_BuildDeviceList() call.",
    8: "FT_IncorrectDevice - The device detected does not match that expected.",
    16: "FT_NoDLLLoaded - The library for this device could not be found.",
    17: "FT_NoFunctionsAvailable - No functions available for this device.",
    18: "FT_FunctionNotAvailable - The function is not available for this device.",
    19: "FT_BadFunctionPointer - Bad function pointer detected.",
    20: "FT_GenericFunctionFail - The function failed to complete successfully.",
    21: "FT_SpecificFunctionFail - The function failed to complete successfully.",
    32: "TL_ALREADY_OPEN - Attempt to open a device that was already open.",
    33: "TL_NO_RESPONSE - The device has stopped responding.",
    34: "TL_NOT_IMPLEMENTED - This function has not been implemented.",
    35: "TL_FAULT_REPORTED - The device has reported a fault.",
    36: "TL_INVALID_OPERATION - The function could not be completed at this time.",
    37: "TL_UNHOMED - The device cannot perform this function until it has been Homed.",
    38: "TL_INVALID_POSITION - The function cannot be performed as it would result in an illegal position.",
    39: "TL_INVALID_VELOCITY_PARAMETER - An invalid velocity parameter was supplied. The velocity must be greater than zero.",
    41: "TL_FIRMWARE_BUG - The firmware has thrown an error.",
    42: "TL_INITIALIZATION_FAILURE - The device has failed to initialize.",
    43: "TL_INVALID_CHANNEL - An Invalid channel address was supplied.",
    44: "TL_CANNOT_HOME_DEVICE - This device does not support Homing. Check the Limit switch parameters are correct.",
    45: "TL_JOG_CONTINUOUS_MODE - An invalid jog mode was supplied for the jog function.",
    46: "TL_NO_MOTOR_INFO - There is no Motor Parameters available to convert Real World Units.",
    47: "TL_CMD_TEMP_UNAVAILABLE - Command temporarily unavailable, Device may be busy."
}


class KinesisMotor(Base):
    """ This class is implements communication with Thorlabs motors via Kinesis dll

    Example config for copy-paste:

    kinesis:
        module.Class: 'motor.kinesis_motor.KinesisMotor'
        options:
            dll_folder: 'C:\Program Files\Thorlabs\Kinesis'
            serial_numbers: [000000123]
            names: ['phi']

    This hardware file have been develop for the TDC001/KDC101 rotation controller. It should work with other motors
    compatible with kinesis. Please be aware that Kinesis dll can be a little buggy sometimes.
    In particular conversion to real unit is sometimes broken. The following page helped me :
    https://github.com/MSLNZ/msl-equipment/issues/1

    """
    dll_folder = ConfigOption('dll_folder', default=r'C:\Program Files\Thorlabs\Kinesis')
    dll_file = ConfigOption('dll_file', default='Thorlabs.MotionControl.KCube.DCServo.dll')
    serial_numbers = ConfigOption('serial_numbers', missing='error')
    names = ConfigOption('names', missing='error')
    polling_rate_ms = ConfigOption('polling_rate_ms', default=200)
    
    # =========================================================================
    #                            Basic functions
    # =========================================================================
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._dll = None
        self._codes = None
        self._serial_numbers = None

    def on_activate(self):
        """ Module activation method """
        os.environ['PATH'] = str(self.dll_folder) + os.pathsep + os.environ['PATH']  # needed otherwise dll don't load
        self._dll = ct.cdll.LoadLibrary(self.dll_folder + "\\" + self.dll_file)
        self._dll.TLI_BuildDeviceList()

        self._serial_numbers = {}

        for i, name in enumerate(self.names):
            serial_number = ct.c_char_p(str(self.serial_numbers[i]).encode('utf-8'))
            self._dll.CC_Open(serial_number)
            self._dll.CC_LoadSettings(serial_number)
            self._dll.CC_StartPolling(serial_number, ct.c_int(200))
            self._serial_numbers[name] = serial_number

    def on_deactivate(self):
        """ Disconnect from hardware on deactivation. """
        for name, serial_number in self._serial_numbers.items():
            self._dll.CC_ClearMessageQueue(serial_number)
            self._dll.CC_StopPolling(serial_number)
            self._dll.CC_Close(serial_number)
            
    # =========================================================================
    #                          Commands
    # =========================================================================
    
    def get_position(self, name):
        """ Get the position in real work unit of the motor """
        serial_number = self._serial_numbers[name]
        position = self._dll.CC_GetPosition(serial_number)
        real_unit = ct.c_double()
        self._dll.CC_GetRealValueFromDeviceUnit(serial_number, position, ct.byref(real_unit), 0)
        return real_unit.value

    def set_position(self, name, value):
        """ Set the position in real work unit of an axis """
        serial_number = self._serial_numbers[name]
        device_unit = ct.c_int()
        self._dll.CC_GetDeviceUnitFromRealValue(serial_number, ct.c_double(value), ct.byref(device_unit), 0)
        self._dll.CC_MoveToPosition(serial_number, device_unit)

    def home(self, name):
        """ Send a home instruction to a motor """
        serial_number = self._serial_numbers[name]
        self._dll.CC_Home(serial_number)

    def get_velocity(self, name):
        """ Get the velocity in real work unit of the motor """
        serial_number = self._serial_numbers[name]
        acceleration = ct.c_int()
        max_velocity = ct.c_int()
        real_unit = ct.c_double()
        self._dll.CC_GetVelParams(serial_number, ct.byref(acceleration), ct.byref(max_velocity)) 
        self._dll.CC_GetRealValueFromDeviceUnit(serial_number, max_velocity, ct.byref(real_unit), 1)
        return real_unit.value
