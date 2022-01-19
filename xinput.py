# -*- coding: utf-8 -*-

"""
This hardware module implement the joystick interface to use an Xbox 360 or xbox one controller.
This use the xpinut api and only works for Windows

This module can be used with emulator like x360ce for non xinput controllers
"""
import ctypes


class JoystickXInput():

    _joystick_id = 0
    _dll_path = None
    _axis_maximum = 32767
    _trigger_maximum = 255

    def __init__(self):
        try:
            if self._dll_path is None:
                self._dll = ctypes.windll.xinput9_1_0
            else:
                self._dll = ctypes.WinDLL(self._dll_path)
        except OSError:
            _dll = None
            print("Can not load xinput dll")

    def is_connected(self):
        """ Return true if the joystick is connected without error

        @return boolean: Ok ?
        """
        ERROR_SUCCESS = 0x00000000
        XINPUT_FLAG_GAMEPAD = 0x00000001

        status = self._dll.XInputGetCapabilities(ctypes.c_long(self._joystick_id), XINPUT_FLAG_GAMEPAD,
                                                 ctypes.c_void_p())
        return status == ERROR_SUCCESS

    def get_state(self):
        """ Retrieve the state of the controller
        """
        state = XINPUT_STATE()
        self._dll.XInputGetState(ctypes.c_long(self._joystick_id), ctypes.byref(state))

        bitmasks = {
            'XINPUT_GAMEPAD_DPAD_UP': 0x00000001,
            'XINPUT_GAMEPAD_DPAD_DOWN': 0x00000002,
            'XINPUT_GAMEPAD_DPAD_LEFT': 0x00000004,
            'XINPUT_GAMEPAD_DPAD_RIGHT': 0x00000008,
            'XINPUT_GAMEPAD_START': 0x00000010,
            'XINPUT_GAMEPAD_BACK': 0x00000020,
            'XINPUT_GAMEPAD_LEFT_THUMB': 0x00000040,
            'XINPUT_GAMEPAD_RIGHT_THUMB': 0x00000080,
            'XINPUT_GAMEPAD_LEFT_SHOULDER': 0x0100,
            'XINPUT_GAMEPAD_RIGHT_SHOULDER': 0x0200,
            'XINPUT_GAMEPAD_A': 0x1000,
            'XINPUT_GAMEPAD_B': 0x2000,
            'XINPUT_GAMEPAD_X': 0x4000,
            'XINPUT_GAMEPAD_Y': 0x8000
        }

        state = state.Gamepad
        value_buttons = state.wButtons

        return {'axis': {
            'left_vertical': state.sThumbLY / self._axis_maximum,
            'left_horizontal': state.sThumbLX / self._axis_maximum,
            'right_vertical': state.sThumbRY / self._axis_maximum,
            'right_horizontal': state.sThumbRX / self._axis_maximum,
            'left_trigger': state.bLeftTrigger / self._trigger_maximum,
            'right_trigger': state.bRightTrigger / self._trigger_maximum
            },
         'buttons': {
            'left_up': bool(value_buttons & bitmasks['XINPUT_GAMEPAD_DPAD_UP']),
            'left_down': bool(value_buttons & bitmasks['XINPUT_GAMEPAD_DPAD_DOWN']),
            'left_left': bool(value_buttons & bitmasks['XINPUT_GAMEPAD_DPAD_LEFT']),
            'left_right': bool(value_buttons & bitmasks['XINPUT_GAMEPAD_DPAD_RIGHT']),
            'left_joystick': bool(value_buttons & bitmasks['XINPUT_GAMEPAD_LEFT_THUMB']),

            'right_up': bool(value_buttons & bitmasks['XINPUT_GAMEPAD_Y']),
            'right_down': bool(value_buttons & bitmasks['XINPUT_GAMEPAD_A']),
            'right_left': bool(value_buttons & bitmasks['XINPUT_GAMEPAD_X']),
            'right_right': bool(value_buttons & bitmasks['XINPUT_GAMEPAD_B']),
            'right_joystick': bool(value_buttons & bitmasks['XINPUT_GAMEPAD_RIGHT_THUMB']),

            'middle_left': bool(value_buttons & bitmasks['XINPUT_GAMEPAD_BACK']),
            'middle_right': bool(value_buttons & bitmasks['XINPUT_GAMEPAD_START']),

            'left_shoulder': bool(value_buttons & bitmasks['XINPUT_GAMEPAD_LEFT_SHOULDER']),
            'right_shoulder': bool(value_buttons & bitmasks['XINPUT_GAMEPAD_RIGHT_SHOULDER'])
         }
        }

# Class defined in the xinput API


#class XINPUT_VIBRATION(ctypes.Structure):
#    _fields_ = [
#        ('wLeftMotorSpeed', ctypes.wintypes.WORD),
#        ('wRightMotorSpeed', ctypes.wintypes.WORD)
#    ]


class XINPUT_GAMEPAD(ctypes.Structure):
    _fields_ = [
        ('wButtons', ctypes.c_ushort),
        ('bLeftTrigger', ctypes.c_ubyte),
        ('bRightTrigger', ctypes.c_ubyte),
        ('sThumbLX', ctypes.c_short),
        ('sThumbLY', ctypes.c_short),
        ('sThumbRX', ctypes.c_short),
        ('sThumbRY', ctypes.c_short),
    ]


class XINPUT_STATE(ctypes.Structure):
    _fields_ = [
        ('dwPacketNumber', ctypes.c_ulong),
        ('Gamepad', XINPUT_GAMEPAD),
    ]



