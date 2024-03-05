# -*- coding: utf-8 -*-
"""
This file contains the Qudi hardware module for the ESP300.

Copyright (c) 2021, the qudi developers. 
See the AUTHORS.md file at the top-level directory of this
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

import serial
from collections import OrderedDict
from qudi.core.configoption import ConfigOption
from qudi.interface.motor_interface import MotorInterface


class MotorAxisNumber:
    def __init__(self, label, number):
        self.label = label
        self.number = number

class ScannerNewportESP300(MotorInterface):
    """ Hardware class to control the ESP 300 controller sold by Newport.

    Example config for copy-paste:

    newport_esp300:
        module.Class: 'newport_esp300.ScannerNewportESP300'
        options:
            com_port: 'COM4'
            baudrate: 19200
            axis_number:
                x: 1
                y: 2
                z: 3
    """

    # config options
    _com_port = ConfigOption('com_port', 'COM4', missing='warn')
    _baudrate = ConfigOption('baudrate', 19200, missing='warn')

    _axis_number = ConfigOption('axis_number', missing='warn')

    # =========================================================================
    #                            Basic functions
    # =========================================================================

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.esp = NewportESP300(self._com_port, self._baudrate)

        self._current_position = dict()
        self._constraints = None
        
        self.list_keys, self.list_values = [], []
        for keys, values in self._axis_number.items():
            self.list_keys.append(keys)
            self.list_values.append(values)
        self._x_axis = MotorAxisNumber(self.list_keys[0], self.list_values[0])
        self._y_axis = MotorAxisNumber(self.list_keys[1], self.list_values[1])
        self._z_axis = MotorAxisNumber(self.list_keys[2], self.list_values[2])

        # "Hardware" constraints:
        model_and_serial_number = {ax: self.esp.get_stage_model(self._axis_number[ax]) 
                                   for ax in self._axis_number}
        controller_and_firmware_ver = self.esp.get_firmware_version()
        self.log.info(f'Stage {model_and_serial_number} and controller {controller_and_firmware_ver}')

        self._min_pos = {ax: float(self.esp.get_left_travel_limit(self._axis_number[ax])) 
                         for ax in self._axis_number}
        self._max_pos = {ax: float(self.esp.get_right_travel_limit(self._axis_number[ax])) 
                         for ax in self._axis_number}
        self._velocity = {ax: float(self.esp.get_velocity_value(self._axis_number[ax])) 
                          for ax in self._axis_number}
        self._axis_unit = {ax: axis_unit_dict[int(self.esp.get_axis_displacement_units(self._axis_number[ax]))] 
                           for ax in self._axis_number}
        self._min_step = {ax: float(self.esp.get_position_deadband(self._axis_number[ax])) 
                          for ax in self._axis_number}
        self.log.info('Limits: {0} {2} to {1} {2}'.format(self._min_pos, self._max_pos, self._axis_unit['x']))


    def on_activate(self):
        """ Initialisation performed during activation of the module.
        """
        self.set_up_scanner()
        
        self._current_position = {ax: float(self.esp.read_position(self._axis_number[ax])) 
                                  for ax in self._axis_number}
        return
    
    def on_deactivate(self):
        """ De-initialisation performed during deactivation of the module.
        """
        # Turn all 3 axis motors OFF and close serial connection.
        self.esp.turn_motor_off(1)
        self.esp.turn_motor_off(2)
        self.esp.turn_motor_off(3)
        self.esp._serial_connection.close()

    # =========================================================================
    #                         Helper functions
    # =========================================================================

    # def _read_error(self):
    #     """ Read error code from the controller.
    #
    #     @return bool, str:
    #     """
    #     err = self.esp.read_error_code()
    #     if len(err) > 0 and err[0] != '@':
    #         err_str = self.esp.read_error_message()
    #         # self.log.error(f'ESP300 Error {err_str}')
    #         return True, err_str
    #     return False, ''

    def wait_for_motion_done(self):
        motion_done = False
        while motion_done is False:
            status = self.get_status()
            if status['x'] and status['y'] and status['z']:
                motion_done = True

    def _homing_scanner_xy(self):
        self.esp.origin_searching(4, 1)
        self.wait_for_motion_done()
        self.esp.origin_searching(4, 2)
        self.wait_for_motion_done()
    
    def _get_scanner_to_breadboard_origin(self):
        self.move_abs({'x':9})
        self.move_abs({'y':9})
        self.wait_for_motion_done()

    def set_up_scanner(self):
        """ Setting up the scanner hardware for scanning.
        """
        # Turn all 3 axis motors ON.
        self.esp.turn_motor_on(1)
        self.esp.turn_motor_on(2)
        self.esp.turn_motor_on(3)

        # Set all 3 velocities to maximum value.
        self.esp.set_velocity_value(1, 1)
        self.esp.set_velocity_value(1, 2)
        self.esp.set_velocity_value(1, 3)
        
        # Set a backlash compensation of 10 um for the 'x' scanner (here, axis 1 on the controller).
        self.esp.set_backlash_compensation(0.010, 1)
        # Set a backlash compensation of 8 um for the 'y' scanner (here, axis 2 on the controller).
        self.esp.set_backlash_compensation(0.008, 2)
        
    def get_target_pos(self, param_list=None):
        """ Gets desired position of the stage arms.

        @param list param_list: optional, if a specific position of an axis is desired, then the labels of the needed
                                axis should be passed in the param_list. If nothing is passed, then from each axis
                                the position is asked.
        @return dict: with keys being the axis labels and item the desired position.
        """
        pos = {}
        if param_list is not None:
            if self._x_axis.label in param_list:
                pos[self._x_axis.label] = float(self.esp.get_target_position(self._x_axis.number))

            if self._y_axis.label in param_list:
                pos[self._y_axis.label] = float(self.esp.get_target_position(self._y_axis.number))

            if self._z_axis.label in param_list:
                pos[self._z_axis.label] = float(self.esp.get_target_position(self._z_axis.number))

        else:
            pos[self._x_axis.label] = float(self.esp.get_target_position(self._x_axis.number))
            pos[self._y_axis.label] = float(self.esp.get_target_position(self._y_axis.number))
            pos[self._z_axis.label] = float(self.esp.get_target_position(self._z_axis.number))

        return pos


    # =========================================================================
    #                         MotorInterface Commands
    # =========================================================================


    def get_constraints(self):
        """ Retrieve the hardware constrains from the motor device.

        @return dict: dict with constraints for the magnet hardware. These
                      constraints will be passed via the logic to the GUI so
                      that proper display elements with boundary conditions
                      could be made.

        Provides all the constraints for each axis of a motorized stage
        (like total travel distance, velocity, ...)
        Each axis has its own dictionary, where the label is used as the
        identifier throughout the whole module. The dictionaries for each axis
        are again grouped together in a constraints dictionary in the form

            {'<label_axis0>': axis0 }

        where axis0 is again a dict with the possible values defined below. The
        possible keys in the constraint are defined here in the interface file.
        If the hardware does not support the values for the constraints, then
        insert just None. If you are not sure about the meaning, look in other
        hardware files to get an impression.

        Example of how a return dict with constraints might look like:
        ==============================================================

        constraints = {}

        axis0 = {}
        axis0['label'] = 'x'    # it is very crucial that this label coincides
                                # with the label set in the config.
        axis0['unit'] = 'm'     # the SI units, only possible m or degree
        axis0['ramp'] = ['Sinus','Linear'], # a possible list of ramps
        axis0['pos_min'] = 0,
        axis0['pos_max'] = 100,  # that is basically the traveling range
        axis0['pos_step'] = 100,
        axis0['vel_min'] = 0,
        axis0['vel_max'] = 100,
        axis0['vel_step'] = 0.01,
        axis0['acc_min'] = 0.1
        axis0['acc_max'] = 0.0
        axis0['acc_step'] = 0.0

        axis1 = {}
        axis1['label'] = 'phi'   that axis label should be obtained from config
        axis1['unit'] = 'degree'        # the SI units
        axis1['ramp'] = ['Sinus','Trapez'], # a possible list of ramps
        axis1['pos_min'] = 0,
        axis1['pos_max'] = 360,  # that is basically the traveling range
        axis1['pos_step'] = 100,
        axis1['vel_min'] = 1,
        axis1['vel_max'] = 20,
        axis1['vel_step'] = 0.1,
        axis1['acc_min'] = None
        axis1['acc_max'] = None
        axis1['acc_step'] = None

        # assign the parameter container for x to a name which will identify it
        constraints[axis0['label']] = axis0
        constraints[axis1['label']] = axis1
        """
        constraints = OrderedDict()

        axis0 = {'label': self._x_axis.label,
                 'unit': 'mm',
                 'ramp': ['Sinus', 'Linear'],
                 'pos_min': -25,
                 'pos_max': 25,
                 'pos_step': 0.00005,
                 'vel_min': 0,
                 'vel_max': 1,
                 'vel_step': 0.01,
                 'acc_min': 0.1,
                 'acc_max': 0.0,
                 'acc_step': 0.0}

        axis1 = {'label': self._y_axis.label,
                 'unit': 'mm',
                 'ramp': ['Sinus', 'Linear'],
                 'pos_min': -25,
                 'pos_max': 25,
                 'pos_step': 0.00005,
                 'vel_min': 0,
                 'vel_max': 1,
                 'vel_step': 0.01,
                 'acc_min': 0.1,
                 'acc_max': 0.0,
                 'acc_step': 0.0}

        axis2 = {'label': self._z_axis.label,
                 'unit': 'mm',
                 'ramp': ['Sinus', 'Linear'],
                 'pos_min': -25,
                 'pos_max': 25,
                 'pos_step': 0.00005,
                 'vel_min': 0,
                 'vel_max': 1,
                 'vel_step': 0.01,
                 'acc_min': 0.1,
                 'acc_max': 0.0,
                 'acc_step': 0.0}

        # assign the parameter container for x to a name which will identify it
        constraints[axis0['label']] = axis0
        constraints[axis1['label']] = axis1
        constraints[axis2['label']] = axis2

        return constraints

    def move_rel(self, param_dict):
        """ Moves stage in given direction (relative movement)

        @param dict param_dict: dictionary, which passes all the relevant
                                parameters, which should be changed. Usage:
                                 {'axis_label': <the-abs-pos-value>}.
                                 'axis_label' must correspond to a label given
                                 to one of the axis.

        A smart idea would be to ask the position after the movement.
        """
        curr_pos_dict = self.get_pos()
        constraints = self.get_constraints()

        if param_dict.get(self._x_axis.label) is not None:
            move_x = param_dict[self._x_axis.label]
            curr_pos_x = curr_pos_dict[self._x_axis.label]

            if  (curr_pos_x + move_x > constraints[self._x_axis.label]['pos_max'] ) or\
                (curr_pos_x + move_x < constraints[self._x_axis.label]['pos_min']):

                self.log.warning('Cannot make further movement of the axis "{0}" with the step {1}, '
                                 'since the border [{2},{3}] was reached! Ignore command!'
                                 .format(self._x_axis.label, move_x, constraints[self._x_axis.label]['pos_min'],
                                         constraints[self._x_axis.label]['pos_max']))
            else:
                self.esp.move_relative_position(move_x, self._x_axis.number)

        if param_dict.get(self._y_axis.label) is not None:
            move_y = param_dict[self._y_axis.label]
            curr_pos_y = curr_pos_dict[self._y_axis.label]

            if  (curr_pos_y + move_y > constraints[self._y_axis.label]['pos_max'] ) or\
                (curr_pos_y + move_y < constraints[self._y_axis.label]['pos_min']):

                self.log.warning('Cannot make further movement of the axis "{0}" with the step {1}, '
                                 'since the border [{2},{3}] was reached! Ignore command!'
                                 .format(self._y_axis.label, move_y, constraints[self._y_axis.label]['pos_min'],
                                         constraints[self._y_axis.label]['pos_max']))
            else:
                self.esp.move_relative_position(move_y, self._y_axis.number)

        if param_dict.get(self._z_axis.label) is not None:
            move_z = param_dict[self._z_axis.label]
            curr_pos_z = curr_pos_dict[self._z_axis.label]

            if  (curr_pos_z + move_z > constraints[self._z_axis.label]['pos_max'] ) or\
                (curr_pos_z + move_z < constraints[self._z_axis.label]['pos_min']):

                self.log.warning('Cannot make further movement of the axis "{0}" with the step {1}, '
                                 'since the border [{2},{3}] was reached! Ignore command!'
                                 .format(self._z_axis.label, move_z, constraints[self._z_axis.label]['pos_min'],
                                         constraints[self._z_axis.label]['pos_max']))
            else:
                self.esp.move_relative_position(move_z, self._z_axis.number)

    def move_abs(self, param_dict):
        """ Moves stage to absolute position (absolute movement)

        @param dict param_dict: dictionary, which passes all the relevant
                                parameters, which should be changed. Usage:
                                 {'axis_label': <the-abs-pos-value>}.
                                 'axis_label' must correspond to a label given
                                 to one of the axis.
        """
        constraints = self.get_constraints()

        if param_dict.get(self._x_axis.label) is not None:
            desired_pos = param_dict[self._x_axis.label]
            constr = constraints[self._x_axis.label]

            if not(constr['pos_min'] <= desired_pos <= constr['pos_max']):
                self.log.warning('Cannot make absolute movement of the axis "{0}" to position {1}, '
                                 'since it exceeds the limits [{2},{3}] ! Command is ignored!'
                                 .format(self._x_axis.label, desired_pos, constr['pos_min'], constr['pos_max']))
            else:
                self.esp.move_absolute_position(desired_pos, self._x_axis.number)

        if param_dict.get(self._y_axis.label) is not None:
            desired_pos = param_dict[self._y_axis.label]
            constr = constraints[self._y_axis.label]

            if not(constr['pos_min'] <= desired_pos <= constr['pos_max']):
                self.log.warning('Cannot make absolute movement of the axis "{0}" to position {1}, '
                                 'since it exceeds the limits [{2},{3}] ! Command is ignored!'
                                 .format(self._y_axis.label, desired_pos, constr['pos_min'], constr['pos_max']))
            else:
                self.esp.move_absolute_position(desired_pos, self._y_axis.number)

        if param_dict.get(self._z_axis.label) is not None:
            desired_pos = param_dict[self._z_axis.label]
            constr = constraints[self._z_axis.label]

            if not(constr['pos_min'] <= desired_pos <= constr['pos_max']):
                self.log.warning('Cannot make absolute movement of the axis "{0}" to position {1}, '
                                 'since it exceeds the limits [{2},{3}] ! Command is ignored!'
                                 .format(self._z_axis.label, desired_pos, constr['pos_min'], constr['pos_max']))
            else:
                self.esp.move_absolute_position(desired_pos, self._z_axis.number)

    def abort(self):
        """ Stops movement of the stage

        @return int: error code (0:OK, -1:error)
        """
        self.esp.abort_motion()
        return 0

    def get_pos(self, param_list=None):
        """ Gets current position of the stage arms.

        @param list param_list: optional, if a specific position of an axis is desired, then the labels of the needed
                                axis should be passed in the param_list. If nothing is passed, then from each axis
                                the position is asked.
        @return dict: with keys being the axis labels and item the current position.
        """
        pos = {}
        if param_list is not None:
            if self._x_axis.label in param_list:
                pos[self._x_axis.label] = float(self.esp.read_position(self._x_axis.number))

            if self._y_axis.label in param_list:
                pos[self._y_axis.label] = float(self.esp.read_position(self._y_axis.number))

            if self._z_axis.label in param_list:
                pos[self._z_axis.label] = float(self.esp.read_position(self._z_axis.number))

        else:
            pos[self._x_axis.label] = float(self.esp.read_position(self._x_axis.number))
            pos[self._y_axis.label] = float(self.esp.read_position(self._y_axis.number))
            pos[self._z_axis.label] = float(self.esp.read_position(self._z_axis.number))

        return pos

    def get_status(self, param_list=None):
        """ Get the status of the position

        @param list param_list: optional, if a specific status of an axis
                                is desired, then the labels of the needed
                                axis should be passed in the param_list.
                                If nothing is passed, then from each axis the
                                status is asked.

        @return dict: with the axis label as key and the status number as item.
        """
        status = {}
        if param_list is not None:
            if self._x_axis.label in param_list:
                pass
                status[self._x_axis.label] = int(self.esp.get_axis_motion_status(self._x_axis.number))

            if self._y_axis.label in param_list:
                status[self._y_axis.label] = int(self.esp.get_axis_motion_status(self._y_axis.number))

            if self._z_axis.label in param_list:
                status[self._z_axis.label] = int(self.esp.get_axis_motion_status(self._z_axis.number))

        else:
            status[self._x_axis.label] = int(self.esp.get_axis_motion_status(self._x_axis.number))
            status[self._y_axis.label] = int(self.esp.get_axis_motion_status(self._y_axis.number))
            status[self._z_axis.label] = int(self.esp.get_axis_motion_status(self._z_axis.number))

        return status

    def calibrate(self, param_list=None):
        """ Calibrates the stage.

        @param dict param_list: param_list: optional, if a specific calibration
                                of an axis is desired, then the labels of the
                                needed axis should be passed in the param_list.
                                If nothing is passed, then all connected axis
                                will be calibrated.

        @return int: error code (0:OK, -1:error)

        After calibration the stage moves to home position which will be the
        zero point for the passed axis. The calibration procedure will be
        different for each stage.
        """
        if param_list is not None:
            if self._x_axis.label in param_list:
                self.esp.origin_searching(4, self._x_axis.number)

            if self._y_axis.label in param_list:
                self.esp.origin_searching(4, self._y_axis.number)

            if self._z_axis.label in param_list:
                self.esp.origin_searching(4, self._z_axis.number)

        else:
            self.esp.origin_searching(4, self._x_axis.number)
            self.esp.origin_searching(4, self._y_axis.number)
            self.esp.origin_searching(4, self._z_axis.number)

        return 0

    def get_velocity(self, param_list=None):
        """ Gets the current velocity for all connected axes.

        @param dict param_list: optional, if a specific velocity of an axis
                                is desired, then the labels of the needed
                                axis should be passed as the param_list.
                                If nothing is passed, then from each axis the
                                velocity is asked.

        @return dict : with the axis label as key and the velocity as item.
        """
        vel = {}
        if param_list is not None:
            if self._x_axis.label in param_list:
                vel[self._x_axis.label] = float(self.esp.get_velocity(self._x_axis.number))

            if self._y_axis.label in param_list:
                vel[self._x_axis.label] = float(self.esp.get_velocity(self._y_axis.number))

            if self._z_axis.label in param_list:
                vel[self._x_axis.label] = float(self.esp.get_velocity(self._z_axis.number))

        else:
            vel[self._x_axis.label] = float(self.esp.get_velocity(self._x_axis.number))
            vel[self._y_axis.label] = float(self.esp.get_velocity(self._y_axis.number))
            vel[self._z_axis.label] = float(self.esp.get_velocity(self._z_axis.number))

        return vel

    def set_velocity(self, param_dict):
        """ Write new value for velocity.

        @param dict param_dict: dictionary, which passes all the relevant
                                parameters, which should be changed. Usage:
                                 {'axis_label': <the-velocity-value>}.
                                 'axis_label' must correspond to a label given
                                 to one of the axis.
        """
        constraints = self.get_constraints()

        if param_dict.get(self._x_axis.label) is not None:
            desired_vel = param_dict[self._x_axis.label]
            constr = constraints[self._x_axis.label]

            if not(constr['vel_min'] <= desired_vel <= constr['vel_max']):
                self.log.warning('Cannot set velocity of the axis "{0}" to position {1}, '
                                 'since it exceeds the limits [{2},{3}] ! Command is ignored!'
                                 .format(self._x_axis.label, desired_vel, constr['vel_min'], constr['vel_max']))
            else:
                self.esp.set_velocity_value(desired_vel, self._x_axis.number)

        if param_dict.get(self._y_axis.label) is not None:
            desired_vel = param_dict[self._y_axis.label]
            constr = constraints[self._y_axis.label]

            if not(constr['vel_min'] <= desired_vel <= constr['vel_max']):
                self.log.warning('Cannot set velocity of the axis "{0}" to position {1}, '
                                 'since it exceeds the limits [{2},{3}] ! Command is ignored!'
                                 .format(self._y_axis.label, desired_vel, constr['vel_min'], constr['vel_max']))
            else:
                self.esp.set_velocity_value(desired_vel, self._y_axis.number)

        if param_dict.get(self._z_axis.label) is not None:
            desired_vel = param_dict[self._z_axis.label]
            constr = constraints[self._z_axis.label]

            if not(constr['vel_min'] <= desired_vel <= constr['vel_max']):
                self.log.warning('Cannot set velocity of the axis "{0}" to possition {1}, '
                                 'since it exceeds the limits [{2},{3}] ! Command is ignored!'
                                 .format(self._z_axis.label, desired_vel, constr['pos_min'], constr['pos_max']))
            else:
                self.esp.set_velocity_value(desired_vel, self._z_axis.number)


    # =========================================================================
    #                           HARDWARE "API"
    # =========================================================================

ERROR_CODE = {
    0: "NO ERROR DETECTED",                     1: "PCI COMMUNICATION TIME-OUT",
    4: "EMERGENCY SOP ACTIVATED",               6: "COMMAND DOES NOT EXIST",
    7: "PARAMETER OUT OF RANGE",                8: "CABLE INTERLOCK ERROR",
    9: "AXIS NUMBER OUT OF RANGE",              13: "GROUP NUMBER MISSING",
    14: "GROUP NUMBER OUT OF RANGE",            15: "GROUP NUMBER NOT ASSIGNED",
    16: "GROUP NUMBER ALREADY ASSIGNED",        17: "GROUP AXIS OUT OF RANGE",
    18: "GROUP AXIS ALREADY ASSIGNED",          19: "GROUP AXIS DUPLICATED",
    20: "DATA ACQUISITION IS BUSY",             21: "DATA ACQUISITION SETUP ERROR",
    22: "DATA ACQUISITION NOT ENABLED",         23: "SERVO CYCLE TICK FAILURE",
    25: "DOWNLOAD IN PROGRESS",                 26: "STORED PROGRAM NOT STARTED",
    27: "COMMAND NOT ALLOWED",                  28: "STORED PROGRAM FLASH AREA FULL",
    29: "GROUP PARAMETER MISSING",              30: "GROUP PARAMETER OUT OF RANGE",
    31: "GROUP MAXIMUM VELOCITY EXCEEDED",      32: "GROUP MAXIMUM ACCELERATION EXCEEDED",
    33: "GROUP MAXIMUM DECELERATION EXCEEDED",  34: "GROUP MOVE NOT ALLOWED DURING MOTION",
    35: "PROGRAM NOT FOUND",                    37: "AXIS NUMBER MISSING",
    38: "COMMAND PARAMETER MISSING",            39: "PROGRAM LABEL NOT FOUND",
    40: "LAST COMMAND CANNOT BE REPEATED",      41: "MAX NUMBER OF LABELS PER PROGRAM EXCEEDED"}

ERROR_CODE_BY_AXIS = {
    0: "MOTOR TYPE NOT DEFINED",                1: "PARAMETER OUT OF RANGE",
    2: "AMPLIFIER FAULT DETECTED",              3: "FOLLOWING ERROR THRESHOLD EXCEEDED",
    4: "POSITIVE HARDWARE LIMIT DETECTED",      5: "NEGATIVE HARDWARE LIMIT DETECTED",
    6: "POSITIVE SOFTWARE LIMIT DETECTED",      7: "NEGATIVE SOFTWARE LIMIT DETECTED",
    8: "MOTOR / STAGE NOT CONNECTED",           9: "FEEDBACK SIGNAL FAULT DETECTED",
    10: "MAXIMUM VELOCITY EXCEEDED",            11: "MAXIMUM ACCELERATION EXCEEDED",
    13: "MOTOR NOT ENABLED",                    15: "MAXIMUM JERK EXCEEDED",
    16: "MAXIMUM DAC OFFSET EXCEEDED",          17: "ESP CRITICAL SETTINGS ARE PROTECTED",
    18: "ESP STAGE DEVICE ERROR",               19: "ESP STAGE DATA INVALID",
    20: "HOMING ABORTED",                       21: "MOTOR CURRENT NOT DEFINED",
    22: "UNIDRIVE COMMUNICATIONS ERROR",        23: "UNIDRIVE NOT DETECTED",
    24: "SPEED OUT OF RANGE",                   25: "INVALID TRAJECTORY MASTER AXIS",
    26: "PARAMETER CHARGE NOT ALLOWED",         27: "INVALID TRAJECTORY MODE FOR HOMING",
    28: "INVALID ENCODER STEP RATIO",           29: "DIGITAL I/O INTERLOCK DETECTED",
    30: "COMMAND NOT ALLOWED DURING HOMING",    31: "COMMAND NOT ALLOWED DUE TO GROUP ASSIGNMENT",
    32: "INVALID TRAJECTORY MODE FOR MOVING"}

axis_unit_dict = {0: 'encoder count', 1: 'motor step', 2: 'millimeter', 
                  3: 'micrometer', 4: 'inches', 5: 'milli-inches', 
                  6: 'micro-inches', 7: 'degree', 8: 'gradient', 
                  9: 'radian', 10: 'milliradian', 11: 'microradian'}

class NewportESP300:
    """ Class to control the ESP 300 controller sold by Newport.
    """
    def __init__(self, com_port, baudrate):
        self._serial_connection = serial.Serial(port=com_port, 
                                                baudrate=baudrate, 
                                                bytesize=8, 
                                                parity='N', 
                                                stopbits=1, 
                                                rtscts=True)

    # =========================================================================
    #                        Hardware Helper functions
    # =========================================================================

    def _write_global_command(self, command):
        """ Write a single command for all axis to the controller.

        @param str command: two-letter command for controller.
        """
        cmd = f'{command}\r\n'.encode('ascii')
        self._serial_connection.write(cmd)

    def _write_global_command_value(self, command, value=""):
        """ Write a single command for all axis to the controller.

        @param str command: two-letter command for controller.
        @param str value: value to write to controller (associated with the command).
        """
        cmd = f'{command}{value}\r\n'.encode('ascii')
        self._serial_connection.write(cmd)

    def _write_command(self, command, axis_number=0):
        """ Write a single command for a specified axis to the controller.

        @param str command: two-letter command for controller.
        @param int axis_number: axis number.
        """
        cmd = f'{axis_number:d}{command}\r\n'.encode('ascii')
        self._serial_connection.write(cmd)

    def _write_command_value(self, command, axis_number=0, value=""):
        """ Write a value associated to a single command for a specified axis to the controller.

        @param str command: two-letter command/variable for controller.
        @param int axis_number: axis number.
        @param str value: value to write to controller (associated with the command).
        """
        cmd = f'{axis_number:d}{command}{value}\r\n'.encode('ascii')
        self._serial_connection.write(cmd)

    def _read(self):
        """ Read an answer from the controller.

        @return str: answer from controller.
        """
        ret = self._serial_connection.read_until(b'\r\n')
        return ret[0:].decode('ascii').rstrip()

    # =========================================================================
    #                           Hardware functions
    # =========================================================================        

    def read_error_message(self):
        """ Read the error message.

        @return str: aa, bb, cc where: aa = error code, bb = timestamp and cc = error message.
        """
        self._write_global_command('TB?')
        return self._read()

    def read_error_code(self):
        """ Read the error code.

        @return int: error code number.
        """
        self._write_global_command('TE?')
        return self._read()

    def turn_motor_on(self, axis_number=None):
        """ Turn motor power ON for the specified axis.

        @param int axis_number: axis number.
        """
        self._write_command('MO', axis_number)

    def turn_motor_off(self, axis_number=None):
        """ Turn motor power OFF for the specified axis.

        @param int axis_number: axis number.
        """
        self._write_command('MF', axis_number)

    def get_motor_status(self, axis_number=None):
        """ Get motor status for the specified axis.

        @return int: motor status:  1: motor power is ON.
                                    0: motor power is OFF.
        """
        self._write_command('MF?', axis_number)
        return self._read()

    def update_motor_driver_settings(self, axis_number=None):
        """ Update Newport programmable driver (i.e., Unidrive) settings into working registers.
        Note: This command should not be issued during motion since the motor power is automatically turned OFF.

        @param int axis_number: axis number.
        """
        self._write_command('QD', axis_number)

    def reset_controller(self):
        """ Hardware reset of the controller. It performs the following preliminary tasks before resetting
        the controller:
        1. Stop all the axes that are in motion. The deceleration value specified using the command AG is used
        to stop the axes.
        2. Wait for 500 ms to allow the axes to settle.
        3. Disable all the axes by turning the power OFF.
        4. Reset to the controller card.
        Once the command to reset the controller is detected by the DSP, the controller will stay in reset for a
        minimum of 200 ms. After the reset condition has occurred (i.e., after the 200 ms reset time), the controller
        firmware reboots the controller. At this point, all the parameters last saved to the non-volatile flash
        memory on the controller will be restored. Furthermore, the controller will detect any stages
        (ESP compatible or otherwise) and drivers connected to the controller. This process can take anywhere up
        to 20 seconds depending upon the controller configuration.
        """
        self._write_global_command('RS')

    def abort_program(self):
        """ Interrupt a motion program in execution. It will not stop a motion in progress.
        It will only stop the program after the current command line finished executing.
        """
        self._write_global_command('AP')

    def set_position_display_resolution(self, resolution, axis_number=None):
        """ Set the display resolution of position information.

        @param int resolution: display resolution, range: 0 to 7.
        @param int axis_number: axis number.
        """
        if resolution < 0 or resolution > 7:
            raise ValueError(f'Resolution({resolution}) must be 0 to 7')
        self._write_command_value('FP', axis_number, resolution)

    def get_position_display_resolution(self, axis_number=None):
        """ Get the display resolution of position information.

        @param int axis_number: axis number.
        @return int: current display resolution.
        """
        self._write_command('FP?', axis_number)
        return self._read()

    def set_dac_offset(self, channel_number=None, offset=0):
        """ Set the DAC offset compensation for the specified DAC channel.
        For the ESP300, there are two DAC channels associated with every axis:
        DAC channels 1 and 2 are associated with axis #1, DAC channels 3 and 4 with axis #2 etc.
        For the DAC offset to take effect, this command must be followed by the ASCII command, UF (Update Filter).

        @param int channel_number: DAC channel number.
        @param float offset: DAC offset value, range: -10.0 to 10.0 Volts.
        """
        self._write_command_value('DO', channel_number, offset)

    def get_dac_offset(self, channel_number=None):
        """ Get the DAC offset compensation for the specified DAC channel.

        @param int channel_number: DAC channel number.
        @return float: current DAC offset value.
        """
        self._write_command('DO?', channel_number)
        return self._read()

    def set_trajectory_mode(self, mode, axis_number=None):
        """ Set the trajectory mode for the specified axis.
        Changing trajectory during motion is not allowed.

        @param int mode: home mode, with:   1: trapezoidal mode
                                            2: s-curve mode
                                            3: jog mode
                                            4: slave to master’s desired position (trajectory)
                                            5: slave to master’s actual position (feedback)
                                            6: slave to master’s actual velocity for jogging
        @param int axis_number: axis number.
        """
        if mode < 1 or mode > 6:
            raise ValueError(f'Mode({mode}) must be in 1 to 6')
        self._write_command_value('TJ', axis_number, mode)

    def get_trajectory_mode(self, axis_number=None):
        """ Get the trajectory mode for the specified axis.

        @param int axis_number: axis number.
        @return int: current trajectory mode.
        """
        self._write_command('TJ?', axis_number)
        return self._read()

    def set_amplifier_io_configuration(self, config, axis_number=None):
        """ Set the amplifier I/O polarity, fault checking, and event handling for the specified axis.
        See the ESP300 user manual for details on the hexadecimal config commands.
        Note: If bit-0 or both bits-1 and -2 are set to zero(0) then no action will be taken by the controller.

        @param int config: amplifier I/O configuration, range: 0 to 0FFFFH (hexadecimal with leading zero(0)).
        @param int axis_number: axis number.
        """
        self._write_command_value('ZA', axis_number, config)

    def get_amplifier_io_configuration(self, axis_number=None):
        """ Get the amplifier I/O polarity, fault checking, and event handling for the specified axis.

        @param int axis_number: axis number.
        @return int: current amplifier I/O configuration in hexadecimal notation.
        """
        self._write_command('ZA?', axis_number)
        return self._read()

    def set_feedback_configuration(self, config, axis_number=None):
        """ Set the feedback configuration, fault checking, and event handling, as well as
        stepper closed-loop positioning for the specified axis.
        See the ESP300 user manual for details on the hexadecimal config commands.
        Note: If bit-0 or both bits-1 and -2 are set to zero(0) then no action will be taken by the controller.

        @param int config: feedback configuration, range: 0 to 0FFFFH (hexadecimal with leading zero(0)).
        @param int axis_number: axis number.
        """
        self._write_command_value('ZB', axis_number, config)

    def get_feedback_configuration(self, axis_number=None):
        """ Get the feedback configuration, fault checking, and event handling, as well as
        stepper closed-loop positioning for the specified axis.

        @param int axis_number: axis number.
        @return int: current feedback configuration in hexadecimal notation.
        """
        self._write_command('ZB?', axis_number)
        return self._read()

    def set_e_stop_configuration(self, config, axis_number=None):
        """ Set the emergency stop (e-stop) configuration, fault checking, and event handling for the specified axis.
        See the ESP300 user manual for details on the hexadecimal config commands.
        Note: If bit-0 or both bits-1 and -2 are set to zero(0) then no action will be taken by the controller.

        @param int config: e-stop configuration, range: 0 to 0FFFFH (hexadecimal with leading zero(0)).
        @param int axis_number: axis number.
        """
        self._write_command_value('ZE', axis_number, config)

    def get_e_stop_configuration(self, axis_number=None):
        """ Get the emergency stop (e-stop) configuration, fault checking, and event handling for the specified axis.

        @param int axis_number: axis number.
        @return int: current emergency stop configuration in hexadecimal notation.
        """
        self._write_command('ZE?', axis_number)
        return self._read()

    def set_following_error_configuration(self, config, axis_number=None):
        """ Set the following error configuration, fault checking, and event handling for the specified axis.
        See the ESP300 user manual for details on the hexadecimal config commands.
        Note: If bit-0 or both bits-1 and -2 are set to zero(0) then no action will be taken by the controller.

        @param int config: following error configuration, range: 0 to 0FFFFH (hexadecimal with leading zero(0)).
        @param int axis_number: axis number.
        """
        self._write_command_value('ZF', axis_number, config)

    def get_following_error_configuration(self, axis_number=None):
        """ Get the following error configuration, fault checking, and event handling for the specified axis.

        @param int axis_number: axis number.
        @return int: current following error configuration in hexadecimal notation.
        """
        self._write_command('ZF?', axis_number)
        return self._read()

    def set_hardware_limit_configuration(self, config, axis_number=None):
        """ Set the hardware limit checking, polarity, and event handling for the specified axis.
        See the ESP300 user manual for details on the hexadecimal config commands.
        Note: If bit-0 or both bits-1 and -2 are set to zero(0) then no action will be taken by the controller.

        @param int config: hardware limit configuration, range: 0 to 0FFFFH (hexadecimal with leading zero(0)).
        @param int axis_number: axis number.
        """
        self._write_command_value('ZH', axis_number, config)

    def get_hardware_limit_configuration(self, axis_number=None):
        """ Get the hardware limit checking, polarity, and event handling for the specified axis.

        @param int axis_number: axis number.
        @return int: current hardware configuration in hexadecimal notation.
        """
        self._write_command('ZH?', axis_number)
        return self._read()

    def set_software_limit_configuration(self, config, axis_number=None):
        """ Set the software limit checking, polarity, and event handling for the specified axis.
        See the ESP300 user manual for details on the hexadecimal config commands.
        Note: If bit-0 or both bits-1 and -2 are set to zero(0) then no action will be taken by the controller.

        @param int config: software limit configuration, range: 0 to 0FFFFH (hexadecimal with leading zero(0)).
        @param int axis_number: axis number.
        """
        self._write_command_value('ZS', axis_number, config)

    def get_software_limit_configuration(self, axis_number=None):
        """ Get the software limit checking, polarity, and event handling for the specified axis.

        @param int axis_number: axis number.
        @return int: current software configuration in hexadecimal notation.
        """
        self._write_command('ZS?', axis_number)
        return self._read()

    def get_esp_system_configuration(self):
        """ Get the present ESP system stage/driver configuration. After each system reset or initialization
        the ESP motion controller detects the presence of Universal drivers and ESP-compatible stages connected.
        See the ESP300 user manual for details on the hexadecimal config commands.

        @return int: current ESP configuration in hexadecimal notation.
        """
        self._write_global_command('ZU')
        return self._read()

    def set_system_configuration(self, config):
        """ Configure system fault checking, event handling and general setup for all axes.
        See the ESP300 user manual for details on the hexadecimal config commands.
        Note: If bit-0 or both bits-1 and -2 are set to zero(0) then no action will be taken by the controller.

        @param int config: system configuration, range: 0 to 0FFFFH (hexadecimal with leading zero(0)).
        """
        self._write_global_command_value('ZZ', config)

    def get_system_configuration(self, axis_number=None):
        """ Get system configuration fault checking, event handling and general setup for all axes.

        @return int: current system configuration in hexadecimal notation.
        """
        self._write_command('ZZ?', axis_number)
        return self._read()

    # #################### STATUS FUNCTIONS ####################

    def get_firmware_version(self):
        """ Read the controller type and version.

        @return str: "ESP300 Version xx yy"
                     where: xx yy = version and release date.
        """
        self._write_global_command('VE?')
        return self._read().split()

    def get_hardware_status(self):
        """ Get general hardware status for all axes.
        It allows user to observe the various digital input signals as they appear to the controller.
        Refer to the user manual for the hardware status register #1 and #2 tables.

        @return int, int: current status in hexadecimal notation.
        """
        self._write_global_command('PH')
        return self._read()

    def get_controller_status(self):
        """ Read the controller status byte.
        The byte returned is in the form of an ASCII character.
        The ASCII to binary conversion table can be found in the user manual.

        @return ASCII character: representing the status byte.
        """
        self._write_global_command('TS')
        return self._read()

    def get_controller_activity(self):
        """ Read the controller activity register.
        The byte returned is in the form of an ASCII character.
        The ASCII to binary conversion table can be found in the user manual.

        @return ASCII character: representing the status byte.
        """
        self._write_global_command('TX')
        return self._read()

    def read_position(self, axis_number=None):
        """ Read the actual position for the specified axis.

        @param int axis_number: axis number.
        @return float: instantaneous real position of the specified axis, in pre-defined units.
        """
        self._write_command('TP', axis_number)
        return self._read()

    def get_target_position(self, axis_number=None):
        """ Read desired position for the specified axis.

        @param int axis_number: axis number.
        @return float: desired position, in pre-defined units.
        """
        self._write_command('DP?', axis_number)
        return self._read()

    def get_velocity(self, axis_number=None):
        """ Read the actual velocity for the specified axis.
        The command can be sent at any time but its real use is while motion is in progress.

        @return float: actual velocity of the axis in pre-defined units.
        """
        self._write_command('TV', axis_number)
        return self._read()

    def get_working_speed(self, axis_number=None):
        """ Read desired velocity for the specified axis.

        @param int axis_number: axis number.
        @return float: desired velocity of the axis in pre-defined units.
        """
        self._write_command('DV?', axis_number)
        return self._read()

    def get_stage_model(self, axis_number=None):
        """ Read stage model and serial number for the specified axis.

        @param int axis_number: axis number.
        @return str: model number, serial number.
        """
        self._write_command('ID?', axis_number)
        stage_model = self._read()
        return stage_model.split(',')

    def get_axis_motion_status(self, axis_number=None):
        """ Read the motion status for the specified axis.

        @param int axis_number: axis number.
        @return int: motion done status: 1 if motion is done.
                                          0 if motion is in progress.
        """
        self._write_command('MD?', axis_number)
        return self._read()

    # #################### MOTION & POSITION CONTROL ####################

    def abort_motion(self):
        """ This command acts as an emergency stop for all axis. It calls an emergency stop event command ZE
        (e-stop event configuration). By default, axes are configured to turn motor power OFF, however, individual axes
        can be configured to stop using emergency deceleration rate set by AE command and maintain motor power.
        It should be used only as an immediate command, not in a program.
        Note: This command affects all axes, however the action taken is determined by each individual’s axis ZE
        command configuration.
        """
        self._write_global_command('AB')
        # self.log.info('ESP300: Emergency stop initiated.')

    def define_home(self, value, axis_number=None):
        """ Define HOME position for specified axis.
        If value is:    Empty: HOME position is current position.
                        Defined: stepper goes to defined position, then HOME position is current position.

        @param int axis_number: axis number.
        @param float value: value to write to controller associated with the command (can be empty here).
        """
        self._write_command_value('DH', axis_number, value)
        # self.log.info(f'ESP300: New HOME defined for axis {axis_number}.')

    def get_home(self, axis_number=None):
        """ Report the current HOME position for specified axis.

        @param int axis_number: axis number.
        @return float: current HOME position.
        """
        self._write_command('DH?', axis_number)
        return self._read()

    def move_to_hardware_travel_limit(self, direction, axis_number=None):
        """ Move an axis to its limit (positive or negative).
        It uses the home search speed during travel to hardware limit.

        @param int axis_number: axis number.
        @param str direction:   + for positive direction.
                                − for negative direction.
        @return int: motion done status: 1 if motion is done.
                                          0 if motion is in progress.
        """
        if not (direction in ('+', '-')):
            raise ValueError(f'Direction({direction}) must be + or -')
        self._write_command_value('MT', axis_number, direction)
        self._write_command('MT?', axis_number)
        return self._read()

    def move_indefinitely(self, direction, axis_number=None):
        """ Move indefinitely in the specified direction (positive or negative), with the predefined acceleration and
        velocity. Note: Although the command is accepted while a motion is in progress, care should be taken not to
        reverse direction of motion.

        @param int axis_number: axis number.
        @param str direction:   + for positive direction.
                                − for negative direction.
        @return int: motion done status: 1 if motion is done.
                                          0 if motion is in progress.
        """
        if not (direction in ('+', '-')):
            raise ValueError(f'Direction({direction}) must be + or -')
        self._write_command_value('MV', axis_number, direction)
        self._write_command('MV?', axis_number)
        return self._read()

    def move_to_nearest_index(self, direction, axis_number=None):
        """ Move an axis to its nearest index (positive or negative).
        It uses the home search speed during travel to nearest index.

        @param int axis_number: axis number.
        @param str direction:   + for positive direction.
                                − for negative direction.
        @return int: motion done status: 1 if motion is done.
                                          0 if motion is in progress.
        """
        if not (direction in ('+', '-')):
            raise ValueError(f'Direction({direction}) must be + or -')
        self._write_command_value('MZ', axis_number, direction)
        self._write_command('MZ?', axis_number)
        return self._read()

    def origin_searching(self, mode, axis_number=None):
        """ Executes a Home search routine on the specified axis.
        At the end of a home search routine, the position of axes is reset to the value specified using SH command.
        The home search motion status can be monitored with the Motion Done (MD) status command.

        Note: This command should be executed once every time the controller power is turned ON or the controller
        performs a complete system reset. There is no need to issue this command in any other case since the controller
        always keeps track of position, even when the motor power is OFF.

        @param int mode:    0: Find +0 Position Count.
                            1: Find Home and Index Signals.
                            2: Find Home Signal.
                            3: Find Positive Limit Signal.
                            4: Find Negative Limit Signal.
                            5: Find Positive Limit and Index Signals.
                            6: Find Negative Limit and Index Signals.
        @param int axis_number: axis number.
        """
        if mode < 0 or mode > 6:
            raise ValueError(f'Home mode({mode}) must be in 0 to 6')
        self._write_command_value('OR', axis_number, mode)

    def move_absolute_position(self, value, axis_number=None):
        """ Move to an absolute position with the predefined acceleration and velocity.
        Note: Even though the command is accepted while a motion is in progress, care should be taken not to reverse
        direction of motion. When this command is received, the controller verifies if it will produce a change
        of direction.

        @param float value: absolute position destination.
        @param int axis_number: axis number.
        """
        self._write_command_value('PA', axis_number, value)

    def move_relative_position(self, value, axis_number=None):
        """ Move to a relative position at a specified distance unit from the current position, with the predefined
        acceleration and velocity. Note: Even though the command is accepted while a motion is in progress,
        care should be taken not to reverse direction of motion.

        @param float value: relative motion increment.
        @param int axis_number: axis number.
        """
        self._write_command_value('PR', axis_number, value)

    def stop_motion(self, axis_number=None):
        """ Stops a motion in progress using deceleration rate programmed with AG (set deceleration) command on the
        specified axes. If this command is sent with no axis parameter, all axes are stopped.

        @param int axis_number: axis number.
        """
        self._write_command('ST', axis_number)

    # #################### MOTION DEVICE PARAMETERS ####################

    def set_gear_constant(self, gear_constant, axis_number=None):
        """ Set the gear constant for a Newport Unidrive compatible programmable driver for DC servo axis.

        @param float gear_constant: gear constant, range from 0 to 2e9 in revolution / unit of measure.
        @param int axis_number: axis number.

        Note: This command should be used in conjunction with QT (tachometer gain) command. The gear constant is
        defined as the number of revolutions the motor has to make for the motion device to move one displacement.
        This command must be followed by the QD update driver command to take effect.
        """
        self._write_command_value('QG', axis_number, gear_constant)

    def get_gear_constant(self, axis_number=None):
        """ Get the gear constant for a Newport Unidrive compatible programmable driver for DC servo axis.

        @param int axis_number: axis number.
        @return float: current gear constant in revolution / unit of measure.
        """
        self._write_command('QG?', axis_number)
        return self._read()

    def set_tachometer_gain(self, tachometer_gain, axis_number=None):
        """ Set the DC motor tachometer gain for a Newport Unidrive compatible programmable driver axis.

        @param float tachometer_gain: tachometer gain, range from 0 to 20.
        @param int axis_number: axis number.

        Note: This command should be used in conjunction with QG (gear constant) command.
        This command must be followed by the QD update driver command to take effect.
        """
        self._write_command_value('QT', axis_number, tachometer_gain)

    def get_tachometer_gain(self, axis_number=None):
        """ Get the DC motor tachometer gain for a Newport Unidrive compatible programmable driver axis.

        @param int axis_number: axis number.
        @return float: current tachometer gain in Volts/Krpm.
        """
        self._write_command('QT?', axis_number)
        return self._read()

    def set_maximum_following_error_threshold(self, value, axis_number=None):
        """ Set the maximum allowed following error threshold for the specified axis.

        @param float value: maximum allowed following error, range from 0 to (2e9 * encoder resolution).
        @param int axis_number: axis number.
        """
        self._write_command_value('FE', axis_number, value)

    def get_maximum_following_error_threshold(self, axis_number=None):
        """ Get the maximum allowed following error threshold for the specified axis.

        @param int axis_number: axis number.
        @return float: current maximum allowed following error threshold.
        """
        self._write_command('FE?', axis_number)
        return self._read()

    def set_full_step_resolution(self, resolution, axis_number=None):
        """  Set the encoder full step resolution for the specified axis.

        @param float resolution: encoder full step resolution, 2e-9 to 2e+9 in user defined units.
        @param int axis_number: axis number.
        """
        self._write_command_value('FR', axis_number, resolution)

    def get_full_step_resolution(self, axis_number=None):
        """ Get the encoder full step resolution for the specified axis.

        @param int axis_number: axis number.
        @return float: current encoder full step resolution.
        """
        self._write_command('FR?', axis_number)
        return self._read()

    def set_max_motor_current(self, motor_current, axis_number=None):
        """ Set the maximum motor current output for the specified axis.
        This command must be followed by the QD update driver command to take effect.

        @param float motor_current: motor current.
        @param int axis_number: axis number.
        """
        self._write_command_value('QI', axis_number, motor_current)

    def get_max_motor_current(self, axis_number=None):
        """ Get the maximum motor current output for specified axis.

        @param int axis_number: axis number.
        @return float: current maximum motor current output.
        """
        self._write_command('QI?', axis_number)
        return self._read()

    def set_motor_type(self, motor_type, axis_number=None):
        """ Set the motor type for the specified axis. Defining motor type is necessary because the ESP needs to apply
          different control algorithms for different motor types. It will not be possible to control an axis if
          its motor type is undefined.

          @param int motor_type: 0 = motor type undefined (default).
                                1 = DC servo motor (single analog channel).
                                2 = step motor (digital control).
                                3 = commutated step motor (analog control).
                                4 = commutated brushless DC servo motor.
        @param int axis_number: axis number.

        Note: ESP300 motion controller does not support the "step motor (digital control)" motor type.
        """
        if motor_type < 0 or motor_type > 4:
            return ValueError(f'Motor type({motor_type}) must be in 0 to 4')
        self._write_command_value('QM', axis_number, motor_type)

    def get_motor_type(self, axis_number=None):
        """ Get the motor type for the specified axis.

        @param int axis_number: axis number.
        @return int: current motor type.
        """
        self._write_command('QM?', axis_number)
        return self._read()

    def set_reduce_motor_torque(self, delay, percent, axis_number=None):
        """ This command automatically reduces the specified step motor’s current (i.e., torque) output to the
        requested percentage "percent" after motion has stopped and the specified time "delay" has expired.
        The purpose of this command is to help reduce the motor heating typically generated by stepper motors.
        If axis_number is equal to 0, the torque reduction parameters get applied to all axes.

        @param int delay: delay period, 0 to 60000 milliseconds.
        @param float percent: motor current reduction percentage, 0 to 100 percent of max. motor current.
        @param int axis_number: axis number.
        """
        self._write_command_value('QR', axis_number, f'{delay}, {percent}')

    def get_reduce_motor_torque(self, axis_number=None):
        """ Get the torque reduction settings for the specified axis.

        @param int axis_number: axis number.
        @return int, float: torque reduction settings (delay period in msec, motor current reduction in percent).
        """
        self._write_command('QR?', axis_number)
        return self._read()

    def set_microstep_factor(self, factor, axis_number=None):
        """ Set the microstep factor for the specified axis.
        This command must be followed by the QD update driver command to take effect.

        @param int factor: microstep value.
        @param int axis_number: axis number.
        """
        self._write_command_value('QS', axis_number, factor)

    def get_microstep_factor(self, axis_number=None):
        """ Get the microstep factor for the specified axis.

        @param int axis_number: axis number.
        @return int: current microstep factor.
        """
        self._write_command('QS?', axis_number)
        return self._read()

    def set_motor_voltage(self, voltage, axis_number=None):
        """ Set the average motor voltage output for a Newport Unidrive compatible programmable driver axis.
        This command must be followed by the QD update driver command to take effect.

        @param float voltage: motor voltage.
        @param int axis_number: axis number.
        """
        self._write_command_value('QV', axis_number, voltage)

    def get_motor_voltage(self, axis_number=None):
        """ Get the average motor voltage output for the specified axis.

        @param int axis_number: axis number.
        @return float: average motor voltage setting.
        """
        self._write_command('QV?', axis_number)
        return self._read()

    def set_left_travel_limit(self, value, axis_number=None):
        """ Set the value for the negative (left) software travel limit for the specified axis.

        @param float value: left (negative) software limit.
                            range: (-2e9 ∗ encoder resolution) to 0 in predefined motion units.
        @param int axis_number: axis number.
        """
        self._write_command_value('SL', axis_number, value)

    def get_left_travel_limit(self, axis_number=None):
        """ Get the value for the negative (left) software travel limit for the specified axis.

        @param int axis_number: axis number.
        @return float: current negative (left) software travel limit.
        """
        self._write_command('SL?', axis_number)
        return self._read()

    def set_right_travel_limit(self, value, axis_number=None):
        """ Set the value for the positive (right) software travel limit for the specified axis.

        @param float value: right (positive) software limit.
                            range: 0 to (+2e9 ∗ encoder resolution) in predefined motion units.
        @param int axis_number: axis number.
        """
        self._write_command_value('SR', axis_number, value)

    def get_right_travel_limit(self, axis_number=None):
        """ Get the value for the positive (right) software travel limit for the specified axis.

        @param int axis_number: axis number.
        @return float: current positive (right) software travel limit.
        """
        self._write_command('SR?', axis_number)
        return self._read()

    def set_axis_displacement_units(self, unit, axis_number=None):
        """ Set the displacement units for the specified axis.

        @param int unit: displacement units with:
        0: encoder count    1: motor step       2: millimeter       3: micrometer
        4: inches           5: milli-inches     6: micro-inches     7: degree
        8: gradient         9: radian           10: milliradian     11: microradian
        @param int axis_number: axis number.
        """
        if unit < 0 or unit > 11:
            raise ValueError(f'Unit({unit}) must be in 0 to 11')
        self._write_command_value('SN', axis_number, unit)

    def get_axis_displacement_units(self, axis_number=None):
        """ Get the displacement units for the specified axis.

        @param int axis_number: axis number.
        @return int: current displacement units.
        """
        self._write_command('SN?', axis_number)
        return self._read()

    def set_encoder_resolution(self, resolution, axis_number=None):
        """ Set the encoder resolution for the specified axis.
        Note: The encoder resolution can only be changed when encoder feedback is enabled.
        See ZB (set feedback configuration) command.

        @param float resolution: encoder resolution.
                                range: 2e-9 to 2e+9 in user defined units.
        @param int axis_number: axis number.
        """
        self._write_command_value('SU', axis_number, resolution)

    def get_encoder_resolution(self, axis_number=None):
        """ Get the encoder resolution for the specified axis.

        @param int axis_number: axis number.
        @return float: current encoder resolution.
        """
        self._write_command('SU?', axis_number)
        return self._read()

    # #################### PROGRAMMING ####################
    # DL Define label
    # JL Jump to label
    # EO Automatic execution on power on
    # EP Enter program download mode
    # EX Execute stored program
    # LP List program
    # QP Quit program mode
    # XX Delete a stored program

    def save_settings_to_controller(self):
        """ Save system and axis configuration settings from RAM to non-volatile flash memory on the ESP300. """
        self._write_global_command('SM')

    def get_available_program_memory(self):
        """ Get the amount of unused program memory. The controller has 61440 bytes of non-volatile memory
        available for permanently storing programs. This command reports the amount not used.

        @return: available storage space.
        """
        self._write_global_command('XM')
        return self._read()

    # #################### TRAJECTORY DEFINITION ####################

    def set_acceleration(self, value, axis_number=None):
        """ Set the acceleration value for the specified axis.

        @param float value: acceleration value.
                            range: 0 to the maximum acceleration (AU command) in predefined units/second².
        @param int axis_number: axis number.
        """
        max_accel = self.get_maximum_acceleration_and_deceleration()
        for (ax, max_accel_value) in max_accel.iteritems():
            if value > max_accel_value:
                raise ValueError(f'New acceleration ({value}) exceeds max {max_accel_value} for axis {ax}')
        self._write_command_value('AC', axis_number, value)

    def get_acceleration(self, axis_number=None):
        """ Get the acceleration value for the specified axis.

        @param int axis_number: axis number.
        @return float: current acceleration value.
        """
        self._write_command('AC?', axis_number)
        return self._read()

    def set_e_stop_deceleration(self, value, axis_number=None):
        """ Set the e-stop deceleration value for the specified axis.
        Note: E-stop deceleration value cannot be set lower than the normal deceleration value.
        Refer the description of “AG” command for range of deceleration values.

        @param float value: e-stop deceleration value.
                            range: current normal deceleration value to (2e9 * encoder resolution)
                                    in predefined units/second².
        @param int axis_number: axis number.
        """
        self._write_command_value('AE', axis_number, value)

    def get_e_stop_deceleration(self, axis_number=None):
        """ Get the e-stop deceleration value for the specified axis.

        @param int axis_number: axis number.
        @return float: current e-stop deceleration value.
        """
        self._write_command('AE?', axis_number)
        return self._read()

    def set_deceleration(self, value, axis_number=None):
        """ Set the deceleration value for the specified axis.

        @param float value: deceleration value.
                            range: 0 to the maximum deceleration (AU command) in predefined units/second².
        @param int axis_number: axis number.
        """
        max_decel = self.get_maximum_acceleration_and_deceleration()
        for (ax, max_decel_value) in max_decel.iteritems():
            if value > max_decel_value:
                raise ValueError(f'New deceleration ({value}) exceeds max {max_decel_value} for axis {ax}')
        self._write_command_value('AG', axis_number, value)

    def get_deceleration(self, axis_number=None):
        """ Get the deceleration value for the specified axis.

        @param int axis_number: axis number.
        @return float: current deceleration value.
        """
        self._write_command('AG?', axis_number)
        return self._read()

    def set_maximum_acceleration_and_deceleration(self, value, axis_number=None):
        """ Set the maximum acceleration and deceleration value for the specified axis.

        @param float value: maximum acceleration and deceleration value.
                            range: 0 to 2e+9 in predefined units/second².
        @param int axis_number: axis number.
        """
        self._write_command_value('AU', axis_number, value)

    def get_maximum_acceleration_and_deceleration(self, axis_number=None):
        """ Get the maximum acceleration and deceleration value for the specified axis.

        @param int axis_number: axis number.
        @return float: current maximum acceleration and deceleration value.
        """
        self._write_command('AU?', axis_number)
        return self._read()

    def set_backlash_compensation(self, value, axis_number=None):
        """ This command initiates a backlash compensation algorithm when motion direction is reversed.
        The controller keeps track of the motion sequence and for each direction change it adds the specified "value"
        correction. Setting "value" to zero disables the backlash compensation.

        @param float value: backlash compensation value.
                            range: 0 to distance equivalent to 10000 encoder counts in user units.
        @param int axis_number: axis number.
        """
        self._write_command_value('BA', axis_number, value)

    def get_backlash_compensation(self, axis_number=None):
        """ Get the backlash compensation value for the specified axis.

        @param int axis_number: axis number.
        @return float: current backlash compensation value.
        """
        self._write_command('BA?', axis_number)
        return self._read()

    def set_linear_compensation(self, value, axis_number=None):
        """ Allows users to compensate for linear positioning errors due to stage inaccuracies.
        Such errors decrease or increase actual motion linearly over the travel range.
        The linear compensation value is calculated according to the formula:
        value = (error / travel),
        where:  travel = measured travel range,
                error = error accumulated over the measured travel range.

        Note: The command is affective only after a home search (OR) or define home (DH) is performed
        on the specified axis.

        @param float value: linear compensation value.
                            range: 0 to 2e+9.
        @param int axis_number: axis number.
        """
        self._write_command_value('CO', axis_number, value)

    def get_linear_compensation(self, axis_number=None):
        """ Get linear compensation for the specified axis.

        @param int axis_number: axis number.
        @return float: current linear compensation value.
        """
        self._write_command('CO?', axis_number)
        return self._read()

    def set_jog_high_speed(self, value, axis_number=None):
        """ Set the high speed for jogging the specified axis.

        @param float value: high speed value.
                            range: 0 to maximum velocity (VU command) in preset units/second.
        @param int axis_number: axis number.
        """
        self._write_command_value('JH', axis_number, value)

    def get_jog_high_speed(self, axis_number=None):
        """ Get the high speed for jogging the specified axis.

        @param int axis_number: axis number.
        @return float: current jog high speed value.
        """
        self._write_command('JH?', axis_number)
        return self._read()

    def set_jog_low_speed(self, value, axis_number=None):
        """ Set the low speed for jogging the specified axis.

        @param float value: low speed value.
                            range: 0 to maximum velocity (VU command) in preset units/second.
        @param int axis_number: axis number.
        """
        self._write_command_value('JW', axis_number, value)

    def get_jog_low_speed(self, axis_number=None):
        """ Get the low speed for jogging the specified axis.

        @param int axis_number: axis number.
        @return float: current jog low speed value.
        """
        self._write_command('JW?', axis_number)
        return self._read()

    def set_jerk_rate(self, value, axis_number=None):
        """ Set the jerk (i.e., rate of change in acceleration) value for the specified axis.

        @param float value: jerk value.
                            range: 0 to 2e9 preset units/second^3.
        @param int axis_number: axis number.
        """
        self._write_command_value('JK', axis_number, value)

    def get_jerk_rate(self, axis_number=None):
        """ Get the jerk (i.e., rate of change in acceleration) value for the specified axis.

        @param int axis_number: axis number.
        @return float: current jerk value.
        """
        self._write_command('JK?', axis_number)
        return self._read()

    def set_home_search_high_speed(self, value, axis_number=None):
        """ Set the high speed used to search for home location for the specified axis.

        @param float value: high speed value.
                            range: 0 to maximum velocity (VU command) in preset units/second.
        @param int axis_number: axis number.
        """
        self._write_command_value('OH', axis_number, value)

    def get_home_search_high_speed(self, axis_number=None):
        """ Get the high speed used to search for home location for the specified axis.

        @param int axis_number: axis number.
        @return float: current high speed value.
        """
        self._write_command('OH?', axis_number)
        return self._read()

    def set_home_search_low_speed(self, value, axis_number=None):
        """ Set the low speed used to search for home location for the specified axis.

        @param float value: low speed value.
                            range: 0 to maximum velocity (VU command) in preset units/second.
        @param int axis_number: axis number.
        """
        self._write_command_value('OL', axis_number, value)

    def get_home_search_low_speed(self, axis_number=None):
        """ Get the low speed used to search for home location for the specified axis.

        @param int axis_number: axis number.
        @return float: current low speed value.
        """
        self._write_command('OL?', axis_number)
        return self._read()

    def set_home_search_mode(self, mode, axis_number=None):
        """ Select the home search type without invoking the home search sequence.

        @param int mode: home search mode, with:
                          0: +0 Position Count,
                          1: Home Switch and Index Signals,
                          2: Home Switch Signal,
                          3: Positive Limit Signal,
                          4: Negative Limit Signal,
                          5: Positive Limit and Index Signals
                          6: Negative Limit and Index Signals.
        @param int axis_number: axis number.
        """
        self._write_command_value('OM', axis_number, mode)

    def get_home_search_mode(self, axis_number=None):
        """ Get the home search type without invoking the home search sequence.

        @param int axis_number: axis number.
        @return int: current home search mode.
        """
        self._write_command('OM?', axis_number)
        return self._read()

    def set_home_preset_position(self, position, axis_number=None):
        """ Set the value that is loaded in the position counter when home is found.
        The default value for all motion devices is 0.

        @param float position: home preset position.
                                range: any position within the travel limits, in defined motion units.
        @param int axis_number: axis number.
        """
        self._write_command_value('SH', axis_number, position)

    def get_home_preset_position(self, axis_number=None):
        """ Get the value that is loaded in the position counter when home is found.

        @param int axis_number: axis number.
        @return float: current home preset position.
        """
        self._write_command('SH?', axis_number)
        return self._read()

    def set_velocity_value(self, velocity_value, axis_number=None):
        """ Set the velocity value for the specified axis.

        @param float velocity_value: velocity value.
                                      range: 0 to maximum velocity (VU command) in preset units/second.
        @param int axis_number: axis number.
        """
        max_vel = self.get_max_velocity(axis_number)
        if velocity_value > float(max_vel):
            raise ValueError(f'New velocity({velocity_value}) exceeds max {max_vel} for axis {axis_number}')
        self._write_command_value('VA', axis_number, velocity_value)

    def get_velocity_value(self, axis_number=None):
        """ Get the velocity value for the specified axis.

        @param int axis_number: axis number.
        @return float: current velocity value.
        """
        self._write_command('VA?', axis_number)
        return self._read()

    def set_base_velocity_for_step_motors(self, base_velocity_value, axis_number=None):
        """ Set the base velocity, also referred to as start/stop velocity value for a step motor driven axis.
        Its execution is immediate, meaning that the velocity is changed when the command is processed, even while a
        motion is in progress. It can be used as an immediate command or inside a program.
        Note: Avoid changing the velocity during the acceleration or deceleration periods. For better predictable
        results, change velocity only when the axis is not moving or when it is moving with a constant speed.

        @param float base_velocity_value: base velocity value.
                                          range: 0 to maximum velocity (VU command) in preset units/second.
        @param int axis_number: axis number.
        """
        max_vel = self._get_max_velocity()
        for (ax, max_vel_value) in max_vel.iteritems():
            if base_velocity_value > max_vel_value:
                raise ValueError(f'New velocity({base_velocity_value}) exceeds max {max_vel_value} for axis {ax}')
        self._write_command_value('VB', axis_number, base_velocity_value)

    def get_base_velocity_for_step_motors(self, axis_number=None):
        """ Get the velocity value for the specified axis.

        @param int axis_number: axis number.
        @return float: current base velocity value.
        """
        self._write_command('VB?', axis_number)
        return self._read()

    def set_max_velocity(self, max_velocity_value, axis_number=None):
        """ Set the maximum velocity value for the specified axis.

        @param float max_velocity_value: maximum velocity value, 0 to 2e+9 in predefined units/second.
        @param int axis_number: axis number.
        """
        self._write_command_value('VU', axis_number, max_velocity_value)

    def get_max_velocity(self, axis_number=None):
        """ Get the maximum velocity value for the specified axis.

        @param int axis_number: axis number.
        @return float: current maximum velocity value.
        """
        self._write_command('VU?', axis_number)
        return self._read()

    # #################### FLOW CONTROL & SEQUENCING ####################
    # RQ Generate service request
    # SA Set device address

    def wait_for_position(self, position, axis_number=None):
        """ Stop program execution until the user specified position is reached.

        @param float position: position value,
                            range: starting position to destination of specified axis, in predefined units.
        @param int axis_number: axis number.
        """
        self._write_command_value('WP', axis_number, position)

    def wait_for_motion_stop(self, delay=0, axis_number=None):
        """ Stop the program execution until a motion is completed. The program is continued only after the
        specified axis reaches its destination. If "axis_number" is not specified, the controller waits for all motion
        in progress to end. If specified "delay" is different from 0, the controller waits an additional "delay"
        in milliseconds after the motion is complete and then executes the next commands.

        @param float delay: delay after motion is complete,
                            range: 0 to 60000 milliseconds.
        @param int axis_number: axis number.
        """
        self._write_command_value('WS', axis_number, delay)

    def wait(self, wait_time=0):
        """ Pause the controller for a specified amount of time.

        @param int wait_time: wait time (delay),
                              range: 0 to 60000 milliseconds.
        """
        self._write_global_command_value('WT', wait_time)

    # #################### I/O FUNCTIONS ####################
    # DC Setup data acquisition
    # DD Get data acquisition done status
    # DE Enable/disable data acquisition
    # DF Get data acquisition sample count
    # DG Get acquisition data
    # ES Define event action command string

    def set_dio_port_direction(self, config):
        """ Set digital I/O (DIO) port A and B direction. Bit-0 corresponds to port A and bit-1 to port B.
        If any bit is set to zero(0) then its corresponding port will become an input only.
        If any bit is set to one(1) then its corresponding port will become an output only.

        @param int config: ports configuration,
                            range: 0 to 03H (hexadecimal with leading zero(0)).
        """
        self._write_global_command_value('BO', config)

    def get_dio_port_direction(self):
        """ Get digital I/O (DIO) port A and B direction.
        Bit-0 corresponds to port A and bit-1 to port B.

        @return int: current ports configuration in hexadecimal notation.
        """
        self._write_global_command('BO?')
        self._read()

    def set_dio_status(self, status):
        """ Set all digital I/O (DIO) port A and B logic level.

        @param int status: port A and B logic level.
                            range: 0 to 0FFFFFFH (hexadecimal with leading zero(0)).
        """
        self._write_global_command_value('SB', status)

    def get_dio_status(self):
        """ Get present status of all digital I/O (DIO) port A and B logic level.

        @return int: current setting in hexadecimal notation.
        """
        self._write_global_command('SB?')
        self._read()

    def wait_for_dio_low(self, bit):
        """ This command causes a program to wait until a selected I/O input bit becomes low. It is level, not edge
        sensitive. This means that at the time of evaluation, if the specified I/O bit is low already, the program
        will continue to execute subsequent commands.

        @param int bit: DIO bit number (0 to 15).
        """
        self._write_command('UL', bit)

    def wait_for_dio_high(self, bit):
        """ This command causes a program to wait until a selected I/O input bit becomes high. It is level, not edge
        sensitive. This means that at the time of evaluation, if the specified I/O bit is low already, the program
        will continue to execute subsequent commands.
        Note: All DIO bits are pulled high on the board. Therefore, a missing signal will cause the wait to complete
        and subsequent commands will continue to be executed.

        @param int bit: DIO bit number (0 to 15).
        """
        self._write_command('UH', bit)

    def set_dio_to_stored_program(self, bit, program=None):
        """ Assign DIO bits for initiating the execution of a desired stored program. Execution of the stored program
        begins when the specified DIO bit changes its state from HIGH to LOW logic level.
        Note: Each DIO bit has a pulled-up resistor to +5V. Therefore, all bits will be at HIGH logic level
        if not connected to external circuit and configured as input.

        @param int bit: a bit number (0 to 15) used to trigger stored program execution.
        @param str program: name of stored program to be executed.
        """
        self._write_command_value('BG', bit, program)

    def get_dio_to_stored_program(self, bit):
        """ Get the stored program assigned to the specified bit.

        @param int bit: a bit number (0 to 15) used to trigger stored program execution.
        @return: current setting.
        """
        self._write_command('BG?', bit)
        self._read()

    def set_dio_to_inhibit_motion(self, bit, level, axis_number=None):
        """ Set DIO bits for inhibiting the motion for the specified axis.

        @param int bit: a bit number (0 to 15) for inhibiting motion.
        @param int level: a bit level (0 = LOW and 1 = HIGH) when axis motion is inhibited.
        @param int axis_number: axis number.
        """
        self._write_command_value('BK', axis_number, f'{bit}, {level}')

    def get_dio_to_inhibit_motion(self, axis_number=None):
        """ Get DIO bits for inhibiting the motion for the specified axis.

        @param int axis_number: axis number.
        @return int, int: current assigned values (bit number, bit level).
        """
        self._write_command('BK?', axis_number)
        self._read()

    def enable_dio_to_inhibit_motion(self, axis_number=None):
        """ Enable motion inhibition for the specified axis through DIO bits.

        @param int axis_number: axis number.
        """
        self._write_command('BL1', axis_number)

    def disable_dio_to_inhibit_motion(self, axis_number=None):
        """  Disable motion inhibition for the specified axis through DIO bits.

        @param int axis_number: axis number.
        """
        self._write_command('BL0', axis_number)

    def get_status_dio_to_inhibit_motion(self, axis_number=None):
        """ Get the motion inhibition status for the specified axis through DIO bits.

        @param int axis_number: axis number.
        @return int: status of inhibiting motion.
        """
        self._write_command('BL?', axis_number)
        self._read()

    def set_dio_to_notify_motion_status(self, bit, level, axis_number=None):
        """ Set DIO bits for notifying the motion status – moving or not moving – for the specified axis.

        @param int bit: a bit number (0 to 15) for notifying motion status.
        @param int level: a bit level (0 = LOW and 1 = HIGH) when axis is not moving.
        @param int axis_number: axis number.
        """
        self._write_command_value('BM', axis_number, f'{bit}, {level}')

    def get_dio_to_notify_motion_status(self, axis_number=None):
        """ Get DIO bits for notifying the motion status – moving or not moving – for the specified axis.

        @param int axis_number: axis number.
        @return int, int: current assigned values (bit number, bit level).
        """
        self._write_command('BM?', axis_number)
        self._read()

    def enable_dio_to_notify_motion_status(self, axis_number=None):
        """ Enable the notification motion status for the specified axis through DIO bits.

        @param int axis_number: axis number.
        """
        self._write_command('BN1', axis_number)

    def disable_dio_to_notify_motion_status(self, axis_number=None):
        """ Disable the notification motion status for the specified axis through DIO bits.

        @param int axis_number: axis number.
        """
        self._write_command('BN0', axis_number)

    def get_status_dio_to_notify_motion_status(self, axis_number=None):
        """ Get the notification motion status for the specified axis through DIO bits.

        @param int axis_number: axis number.
        @return int: status of inhibiting motion.

        """
        self._write_command('BN?', axis_number)
        self._read()

    def set_dio_jog_mode(self, neg_bit, pos_bit, axis_number=None):
        """ Set DIO bits for jogging axes in either negative or positive directions for the specified axis.

        @param int neg_bit: a bit number (0 to 15) for jogging in negative direction.
        @param int pos_bit: a bit number (0 to 15) for jogging in positive direction.
        @param int axis_number: axis number.
        """
        self._write_command_value('BP', axis_number, f'{neg_bit}, {pos_bit}')

    def get_dio_jog_mode(self, axis_number=None):
        """ Get DIO bits for jogging axes in negative and positive directions for the specified axis.

        @param int axis_number: axis number.
        @return int, int: DIO bits used for jogging in negative and positive directions respectively.
        """
        self._write_command('BP?', axis_number)
        self._read()

    def enable_dio_jog_mode(self, axis_number=None):
        """ Enable jogging of a requested axis through DIO bits.

        @param int axis_number: axis number.
        """
        self._write_command('BQ1', axis_number)

    def disable_dio_jog_mode(self, axis_number=None):
        """ Disable jogging of a requested axis through DIO bits.

        @param int axis_number: axis number.
        """
        self._write_command('BQ0', axis_number)

    def get_status_dio_jog_mode(self, axis_number=None):
        """ Get the status of jog through DIO bits.

        @param int axis_number: axis number.
        @return int: the status of jog through DIO bits.
        """
        self._write_command('BQ?', axis_number)
        self._read()

    # #################### GROUP FUNCTIONS ####################
    # HA Set group acceleration
    # HB Read list of groups assigned
    # HC Move group along an arc
    # HD Set group deceleration
    # HE Set group E-stop deceleration
    # HF Group motor power OFF
    # HJ Set group jerk
    # HL Move group along a line
    # HN Create new group
    # HO Group motor power ON
    # HP Get group position
    # HQ Wait for group via point buffer near empty
    # HS Stop group motion
    # HV Set group velocity
    # HW Wait for group motion to stop
    # HX Delete a group
    # HZ Get group size

    # #################### DIGITAL FILTERS ####################

    def set_acceleration_feedforward_gain(self, value, axis_number=None):
        """ Set the acceleration feed-forward gain factor Af for the specified axis.
        It is active for any DC servo based motion device.

        @param float value: acceleration feed-forward gain factor (range: 0 to 2e9).
        @param int axis_number: axis number.
        """
        self._write_command_value('AF', axis_number, value)

    def get_acceleration_feedforward_gain(self, axis_number=None):
        """ Get the acceleration feed-forward gain factor Af for the specified axis.

        @param int axis_number: axis number.
        @return float: current acceleration feed-forward gain factor.
        """
        self._write_command('AF?', axis_number)
        return self._read()

    def set_closed_loop_update_interval(self, value, axis_number=None):
        """ Set the closed loop update interval for the specified axis.

        @param int value: closed loop update interval.
                          range: 0 to 60000 in milliseconds.
        @param int axis_number: axis number.
        If "0" is used as an axis number, this command will set the specified interval to all the axes.
        """
        self._write_command_value('CL', axis_number, value)

    def get_closed_loop_update_interval(self, axis_number=None):
        """ Get the closed loop update interval for the specified axis.

        @param int axis_number: axis number.
        @return int: current closed loop update interval in milliseconds.
        """
        self._write_command('CL?', axis_number)
        return self._read()

    def set_position_deadband(self, value, axis_number=None):
        """ Set the position deadband value for the specified axis.

        @param int value: deadband value (range: 0 to 2e9 in encoder counts).
        @param int axis_number: axis number.
        """
        self._write_command_value('DB', axis_number, value)

    def get_position_deadband(self, axis_number=None):
        """ Get the position deadband value for the specified axis.

        @param int axis_number: axis number.
        @return int: current deadband value in encoder counts.
        """
        self._write_command('DB?', axis_number)
        return self._read()

    def set_derivative_gain(self, derivative_gain, axis_number=None):
        """ Set the derivative gain factor Kd of the PID closed loop.
        It is active for any DC servo based motion device that has been selected to operate in closed loop.
        The command can be sent at any time, but it has no effect until the UF (update filter) is received.

        @param float derivative_gain: derivative gain factor Kd, range from 0 to 2e9.
        @param int axis_number: axis number.
        """
        self._write_command_value('KD', axis_number, derivative_gain)

    def get_derivative_gain(self, axis_number=None):
        """ Get the derivative gain factor Kd of the PID closed loop.

        @param int axis_number: axis number.
        @return float: current derivative gain factor Kd.
        """
        self._write_command('KD?', axis_number)
        return self._read()

    def set_integral_gain(self, integral_gain, axis_number=None):
        """ Set the integral gain factor Ki of the PID closed loop.
        It is active for any DC servo based motion device that has been selected to operate in closed loop.
        The command can be sent at any time, but it has no effect until the UF (update filter) is received.

        @param float integral_gain: integral gain factor Ki, range from 0 to 2e9.
        @param int axis_number: axis number.
        """
        self._write_command_value('KI', axis_number, integral_gain)

    def get_integral_gain(self, axis_number=None):
        """ Get the integral gain factor Ki of the PID closed loop.

        @param int axis_number: axis number.
        @return float: current integral gain factor Ki.
        """
        self._write_command('KI?', axis_number)
        return self._read()

    def set_proportional_gain(self, proportional_gain, axis_number=None):
        """ Set the proportional gain factor Kp of the PID closed loop.
        It is active for any DC servo based motion device that has been selected to operate in closed loop.
        The command can be sent at any time, but it has no effect until the UF (update filter) is received.

        @param float proportional_gain: proportional gain factor Kp, range from 0 to 2e9.
        @param int axis_number: axis number.
        """
        self._write_command_value('KP', axis_number, proportional_gain)

    def get_proportional_gain(self, axis_number=None):
        """ Get the proportional gain factor Kp of the PID closed loop.

        @param int axis_number: axis number.
        @return float: current proportional gain factor Kp.
        """
        self._write_command('KP?', axis_number)
        return self._read()

    def set_saturation_coefficient(self, saturation_level, axis_number=None):
        """ Set the saturation level of the integral factor of the PID closed loop
        and is useful for preventing integral wind-up.
        It is active for any DC servo based motion device that has been selected to operate in closed loop.
        The command can be sent at any time, but it has no effect until the UF (update filter) is received.

        @param float saturation_level: saturation level of integrator Ks, range from 0 to 2e9.
        @param int axis_number: axis number.
        """
        self._write_command_value('KS', axis_number, saturation_level)

    def get_saturation_coefficient(self, axis_number=None):
        """ Get the saturation level of the integral factor of the PID closed loop.

        @param int axis_number: axis number.
        @return float: current saturation level of integrator Ks.
        """
        self._write_command('KS?', axis_number)
        return self._read()

    def update_servo_filter(self, axis_number=0):
        """ Used to make active the latest entered PID parameters.
        Any new value for Kp, Ki, Kd and maximum following error are not being used in the PID loop calculation until
        UF command is received. This assures that the parameters are loaded simultaneously, without any transitional
        glitches in the loop.

        @param int axis_number: axis number.
        If axis_number is missing or set to 0, the controller updates the filters for all axes.
        Otherwise, the controller updates only the filter for the specified axis.
        """
        self._write_command('UF', axis_number)

    def set_velocity_feedforward_gain(self, value, axis_number=None):
        """ Set the velocity feed-forward gain factor Vf for the specified axis.
        It is active for any DC servo based motion device.

        @param float value: velocity feed-forward gain factor (0 to 2e9).
        @param int axis_number: axis number.
        """
        self._write_command_value('VF', axis_number, value)

    def get_velocity_feedforward_gain(self, axis_number=None):
        """ Get the velocity feed-forward gain factor Vf.

        @param int axis_number: axis number.
        @return float: current velocity feed-forward gain factor.
        """
        self._write_command('VF?', axis_number)
        return self._read()

    # #################### MASTER-SLAVE MODE DEFINITION ####################

    def set_ms_gear_ratio(self, reduction_ratio, axis_number=None):
        """ Set the master-slave reduction ratio for a slave axis. The trajectory of the slave is the
        desired trajectory or actual position of the master scaled by reduction ratio.

        @param float reduction_ratio: reduction ratio, range ±1,000,000.
        @param int axis_number: axis number.

        Note: Use this command very carefully. The slave axis will have its speed and acceleration in the same ratio
        as the position. Also, ensure that the ratio used for the slave axis does not cause overflow of
        this axis’ parameters (speed, acceleration), especially with ratios greater than 1.
        """
        self._write_command_value('GR', axis_number, reduction_ratio)

    def get_ms_gear_ratio(self, axis_number=None):
        """ Get the master-slave reduction ratio for a slave axis.

        @param int axis_number: axis number.
        @return float: current reduction ratio.
        """
        self._write_command('GR?', axis_number)
        return self._read()

    def set_ms_jog_velocity_update_interval(self, interval):
        """ Set the jog velocity update interval for slave axis. The jog velocity of slave axis is computed once every
        interval using user specified scaling coefficients and the master axis velocity at the time of computation.
        Note: appropriate trajectory mode has to be specified using TJ command before this command becomes effective.

        @param int interval: jog velocity update interval, 1 to 1000 milliseconds.
        """
        if interval < 1 or interval > 1000:
            raise ValueError(f'Interval({interval}) must be in 1 to 1000')
        self._write_global_command_value('SI', interval)

    def get_ms_jog_velocity_update_interval(self):
        """ Get the jog velocity update interval for slave axis.

        @return int: slave axis jog velocity update interval.
        """
        self._write_global_command('SI?')
        return self._read()

    def set_ms_jog_velocity_scaling_coeff(self, coeff_a, coeff_b):
        """ Set the jog velocity scaling coefficients for slave axis.
        The specified coefficients are used as follows:
        v_s = A v_m + B v_m² sgn(v_m),
        where: v_s is the jog velocity of the slave,
                v_m is the velocity of the master axis.

        @param float coeff_a: Coefficient A.
        @param float coeff_b: Coefficient B.
        """
        self._write_global_command_value('SK', f'{coeff_a}, {coeff_b}')

    def get_ms_jog_velocity_scaling_coeff(self):
        """ Get the jog velocity scaling coefficients for slave axis.
        They refer to the coefficients A and B in:
        v_s = A v_m + B v_m² sgn(v_m),
        where: v_s is the jog velocity of the slave,
                v_m is the velocity of the master axis.

        @return float, float: slave axis jog velocity scaling coefficients (A, B).
        """
        self._write_global_command('SK?')
        return self._read()

    def set_ms_relationship(self, slave_axis, master_axis):
        """ Set master-slave relationship between any two specified axes.
        A few rules are in place for ease of use:
        - The trajectory mode for slave has to be appropriately defined before
        that axis follows master in a desired fashion.
        - An axis cannot be assigned as its own slave if it is already
        in a trajectory mode that is specific to master-slaving.
        - A slave axis cannot be moved individually using PA or PR commands
        if its trajectory mode is specific to master-slaving.

        @param int slave_axis: axis number to be defined as a slave.
        @param int master_axis: axis number to be defined as a master.
        """
        self._write_command_value('SS', slave_axis, master_axis)

    def get_ms_relationship(self, slave_axis):
        """ Get the master axis number for the specified (slave) axis.

        @param int slave_axis: axis number to be defined as a slave.
        @return int: master axis number.
        """
        self._write_command('SS?', slave_axis)
        return self._read()
