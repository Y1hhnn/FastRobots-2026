from enum import Enum

class CMD(Enum):
    PING = 0
    START_RECORD = 1
    STOP_ROBOT = 2
    SEND_LOG = 3
    SET_DURATION=4
    SET_MODE=5
    SET_MOTOR_SCALE=6
    UPDATE_DIST_PID=7
    UPDATE_ORIENT_PID=8
    SET_DIST_SETPOINT=9
    SET_ORIENT_SETPOINT=10
    SET_NAV_SETPOINTS=11
    SET_DIST_SENSOR_MODE=12
    SET_SAMPLE_RATE=13
    SET_MAP_DEGREES=14
    SET_NAV_TARGET=15               # Args: heading_deg|distance_m|seg_id. START_RECORD fires.
    SET_NAV_CALIB=16                # Args: pwm|speed_mps
    RESET_YAW=17                    # No args — zero relative yaw at current heading
    SET_NAV_DIST_MODE=18            # Args: 0 (time) or 1 (KF-integrated)