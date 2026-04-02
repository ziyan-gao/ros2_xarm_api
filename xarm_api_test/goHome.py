import sys
import time
import math
import socket
import struct
from xarm.wrapper import XArmAPI

ip = '192.168.1.232'

arm = XArmAPI(ip, enable_report=True)
arm.motion_enable(enable=True)
arm.set_ft_sensor_enable(0)

arm.clean_error()
arm.clean_warn()
arm.set_ft_sensor_enable(1)
time.sleep(0.5)
arm.set_ft_sensor_zero()

while arm.connected and arm.error_code == 0:
    # ft_raw_force and ft_ext_force will update by reporting socket
    print('raw_force: {}'.format(arm.ft_raw_force))
    print('exe_force: {}'.format(arm.ft_ext_force))

    # # get_ft_sensor_data() will get the last ext_force
    # code, ext_force = arm.get_ft_sensor_data()
    # if code == 0:
    #     print('exe_force: {}'.format(ext_force))
    time.sleep(0.2)

arm.set_ft_sensor_enable(0)
arm.disconnect()