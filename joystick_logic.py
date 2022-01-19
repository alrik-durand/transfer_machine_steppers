# -*- coding: utf-8 -*-

"""
A module for reading a joystick controller via joystick interface.

"""
from PyQt5 import QtCore
import numpy as np

import xinput

class FrameSignal(QtCore.QObject):
    signal = QtCore.pyqtSignal(dict)


class JoystickLogic():

    _axis_threshold = 0.05  # a smaller axis position will not trigger an event
    _compensate_error = True  # activate if the threshold is not enough.
    _fps = 100

    _last_state = None

    _errors = {'left_vertical': 0, 'left_horizontal': 0, 'right_vertical': 0, 'right_horizontal': 0}
    _error_count = 1

    _button_list = ['left_up', 'left_down', 'left_left', 'left_right', 'left_joystick',
               'right_up', 'right_down', 'right_left', 'right_right', 'right_joystick',
               'middle_left', 'middle_right', 'left_shoulder', 'right_shoulder']

    _axis_list = ['left_vertical', 'left_horizontal', 'right_vertical', 'right_horizontal',
                  'left_trigger', 'right_trigger']

    new_frame = FrameSignal()

    def __init__(self):
        self.timer = QtCore.QTimer()
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self.loop)

        self.hardware = xinput.JoystickXInput()

        self._last_state = self.hardware.get_state()
        self.timer.start(int(1000 * 1 / self._fps))

    def loop(self):
        old_state = self._last_state
        state = self.hardware.get_state()
        self._last_state = state

        if self._compensate_error:
            state = self._update_error_statistics(state)

        # First this look at button pressed/released and creates an easily accessible list
        state['pressed_buttons'] = []
        state['released_buttons'] = []
        for button in self._button_list:
            if state['buttons'][button] != old_state['buttons'][button]:
                if state['buttons'][button]:
                    state['pressed_buttons'].append(button)
                else:
                    state['released_buttons'].append(button)

        # Generate a trigger value with trigger value that can be directly accessed
        state['axis']['trigger'] = state['axis']['right_trigger'] - state['axis']['left_trigger']

        self.new_frame.emit(state)
        self.timer.start(int(1000 * 1 / self._fps))


    def _update_error_statistics(self, state):
        """ Function  called internally when a new state is received to record the default value of the axis

         The horizontal and vertical axis have a permanent error.
         One way to reduce this effect is to use a threshold but this often is not enough because of a relativerly
         large systematic error. This idea of the feature is to compute the mean value of the controller axis to
         use this position as the zero.
         """
        axis = state['axis']
        keys = ['left_vertical', 'left_horizontal', 'right_vertical', 'right_horizontal']
        # if all the axis are zero, eithere the controller is perfect and this feature is not needed
        # or the controller must be unplugged so we should not count it in the statistic
        if axis['left_vertical'] == 0 and \
                axis['left_horizontal'] == 0 and \
                axis['right_vertical'] == 0 and \
                axis['right_horizontal'] == 0:
            return state

        for key in keys:
            self._errors[key] = self._errors[key]*((self._error_count-1)/self._error_count) + \
                                axis[key] / self._error_count
        self._error_count += 1

        for key in keys:
            delta = axis[key] - self._errors[key]
            if np.abs(delta) < self._axis_threshold:
                delta = 0
            axis[key] = delta

        return state
