# -*- coding: utf-8 -*-
"""
Created on Thu Nov 30 16:38:02 2017
@author: admin_lagarde

Reworked on Jan 19 22 by Alrik Durand
"""
import sys
import os

import numpy as np
from PyQt5 import QtGui, QtCore, QtWidgets, uic

import smc100_dummy as smc100
import joystick_logic

AXIS_KEYS = "123456"

mot = smc100.SMC100(AXIS_KEYS, "COM6")

# We have to initialize the motors in the first place. Each one will go to an initial position.
for i in range(6):
    mot.set_speed_um(1000.0, AXIS_KEYS[i])
    # If already homed and ready when program start, does nothing.
    mot.home(AXIS_KEYS[i], mot.get_position_um(AXIS_KEYS[i]))


class EndMoveSignal(QtCore.QObject):
    result = QtCore.pyqtSignal(int)


class Move(QtCore.QRunnable):

    def __init__(self, id, steps):
        super(Move, self).__init__()
        self.id = id
        self.steps = steps
        self.signal = EndMoveSignal()

    def run(self):
        mot.move_relative_um(self.steps, AXIS_KEYS[self.id])
        self.signal.result.emit(self.id)


class AxisWidget(QtWidgets.QWidget):
    """ Create a widget based on the *.ui file. """
    def __init__(self):
        # Get the path to the *.ui file
        this_dir = os.path.dirname(__file__)
        ui_file = os.path.join(this_dir, 'axis_widget.ui')
        super().__init__()
        uic.loadUi(ui_file, self)
        self.setMinimumSize(240, 320)


class Main(QtWidgets.QMainWindow):

    position_setpoint_update = QtCore.pyqtSignal()
    position_hardware_update = QtCore.pyqtSignal()

    def __init__(self):

        # Load UI file and show window
        this_dir = os.path.dirname(__file__)
        ui_file = os.path.join(this_dir, 'interface.ui')
        super().__init__()
        uic.loadUi(ui_file, self)

        self.widgets = []
        for i in range(6):
            self.create_widget(i)

        self.pushButton_stop_motion.clicked.connect(self.stop_motion)
        self.show()

        self.threadpool = QtCore.QThreadPool()
        self.position_hardware = [0.]*6
        self.steps_increment = [100.]*6

        self.moving_axis = None

        for axis in range(6):
            pos = self.get_position(axis)
            self.widgets[axis].current_position.setText(str(pos))

        try:
            self.joystick = joystick_logic.JoystickLogic()
            self.joystick.new_frame.connect(self.on_joystick_frame)
        except:
            print('Joystick not working.')

    def create_widget(self, i):
        widget = AxisWidget()
        number = i // 3 + 1
        letter = 'XYZ'[i % 3]
        widget.title_label.setText('Axe {}{}'.format(letter, number))

        self.widgets.append(widget)
        [self.left_frame, self.right_frame][i // 3].addWidget(widget)

        widget.pushButton_forward.clicked.connect(lambda: self.make_steps(i, self.steps_increment[i]))
        widget.pushButton_backward.clicked.connect(lambda: self.make_steps(i, -self.steps_increment[i]))
        widget.step_input.editingFinished.connect(lambda: self.update_steps_value(i, widget))
        widget.speed_input.editingFinished.connect(lambda: self.set_speed(i, widget))

    def update_steps_value(self, i, widget):
        try:
            value = float(widget.step_input.text())
        except ValueError:
            self.textEdit.setTextColor(QtGui.QColor('red'))
            self.textEdit.setText(r"Error: Wrong step value. This is not a number.")
            return

        if 0.1 <= value < 24000:
            self.steps_increment[i] = value
            self.textEdit.setTextColor(QtGui.QColor('green'))
            self.textEdit.setText("No Error")
        else:
            self.show_error('Error: Wrong step value. The value must be in the range [0.1;24000]')

    def make_steps(self, axis, steps):

        position = self.get_position(axis)
        new_position = position + steps
        if not(0 <= new_position < 24000):
            return self.show_error('Can not move to {}, out of bound [0, 24000].')

        if self.moving_axis is not None:
            return self.show_error('Already moving.')

        self.moving_axis = axis

        move_action = Move(axis, steps)
        self.threadpool.start(move_action)
        move_action.signal.result.connect(self.on_end_move)


    def on_end_move(self, axis):
        pos = self.get_position(axis)
        self.widgets[axis].current_position.setText(str(pos))
        self.moving_axis = None

    def set_speed(self, axis, widget):
        try:
            value = float(widget.speed_input.text())
        except ValueError:
            self.textEdit.setTextColor(QtGui.QColor('red'))
            self.textEdit.setText(r"Error: Wrong speed value. This is not a number.")
            return
        if not (0 < value <= 1000):
            return self.show_error('Speed value must be un the range  ]0, 1000].')

        mot.set_speed_um(value, AXIS_KEYS[axis])

    def get_position(self, axis):
        position = mot.get_position_um(AXIS_KEYS[axis])
        return position

    def show_error(self, message):
        self.textEdit.setTextColor(QtGui.QColor('red'))
        self.textEdit.setText(message)

    def closeEvent(self, event):
        mot.close()
        event.accept()
    
    def stop_motion(self):
        mot.stop(self.moving_axis)
    
    def keyPressEvent(self, e):

        if self.moving_axis is not None:
            return

        if e.key() == QtCore.Qt.Key_F5:
            self.close()

        elif e.key() == QtCore.Qt.Key_6:
            self.make_steps(0, self.steps_increment[0])
        elif e.key() == QtCore.Qt.Key_4:
            self.make_steps(0, -self.steps_increment[0])
        elif e.key() == QtCore.Qt.Key_8:
            self.make_steps(2, self.steps_increment[2])
        elif e.key() == QtCore.Qt.Key_5:
            self.make_steps(2, -self.steps_increment[2])
        elif e.key() == QtCore.Qt.Key_Minus:
            self.make_steps(1, self.steps_increment[1])
        elif e.key() == QtCore.Qt.Key_Plus:
            self.make_steps(1, -self.steps_increment[1])
        elif e.key() == QtCore.Qt.Key_D:
            self.make_steps(5, self.steps_increment[5])
        elif e.key() == QtCore.Qt.Key_Q:
            self.make_steps(5, -self.steps_increment[5])
        elif e.key() == QtCore.Qt.Key_Z:
            self.make_steps(4, self.steps_increment[4])
        elif e.key() == QtCore.Qt.Key_S:
            self.make_steps(4, -self.steps_increment[4])
        elif e.key() == QtCore.Qt.Key_T:
            self.make_steps(3, self.steps_increment[3])
        elif e.key() == QtCore.Qt.Key_G:
            self.make_steps(3, -self.steps_increment[3])

    def on_joystick_frame(self, state):

        


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    test = Main()
    app.exec()
