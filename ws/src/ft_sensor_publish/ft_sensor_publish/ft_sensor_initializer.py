import os
import sys
import time

from xarm.wrapper import XArmAPI


def return_code(result):
    if isinstance(result, (tuple, list)):
        return int(result[0])
    return int(result)


def main():
    """Enable the xArm FT sensor once; the ROS driver publishes its reports."""
    robot_ip = os.environ.get('ROBOT_IP', '192.168.1.232')
    arm = XArmAPI(robot_ip, enable_report=False)
    try:
        if not arm.connected:
            print(f'FT sensor initialization failed: cannot connect to {robot_ip}',
                  file=sys.stderr)
            return 1
        result = arm.set_ft_sensor_enable(1)
        if return_code(result) != 0:
            print(f'FT sensor initialization failed: {result}', file=sys.stderr)
            return 1
        time.sleep(0.5)
        result = arm.set_ft_sensor_zero()
        if return_code(result) != 0:
            print(f'FT sensor zeroing failed: {result}', file=sys.stderr)
            return 1
        time.sleep(0.5)
        print(f'FT sensor enabled and zeroed at {robot_ip}')
        return 0
    finally:
        arm.disconnect()


if __name__ == '__main__':
    raise SystemExit(main())
