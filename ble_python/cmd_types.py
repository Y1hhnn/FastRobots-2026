from enum import Enum

class CMD(Enum):
    PING = 0
    START_RECORD = 1
    STOP_ROBOT = 2
    SEND_LOG = 3
    UPDATE_PID = 4
    SET_DURATION = 5
    SET_SETPOINT = 6
    SET_MODE = 7
    SET_MOTOR_SCALE = 8
    SET_EXTRAPOLATION = 9