import os
import sys
import time

from xarm.wrapper import XArmAPI


def return_code(result):
    if isinstance(result, (tuple, list)):
        return int(result[0])
    return int(result)


def main():
    """Recover, enable, and zero the FT sensor before motion nodes start."""
    robot_ip = os.environ.get('ROBOT_IP', '192.168.1.232')
    arm = XArmAPI(robot_ip, enable_report=False)
    try:
        if not arm.connected:
            print(f'FT sensor initialization failed: cannot connect to {robot_ip}',
                  file=sys.stderr)
            return 1
        last_failure = 'unknown failure'
        for attempt in range(1, 3):
            # Match UFACTORY's recovery order. Zeroing is performed only here,
            # before automatic handling starts and with no carried item.
            arm.set_ft_sensor_enable(0)
            arm.clean_error()
            arm.clean_warn()
            time.sleep(0.2)
            result = arm.set_ft_sensor_enable(1)
            if return_code(result) != 0:
                last_failure = f'enable returned {result}'
            else:
                time.sleep(0.5)
                result = arm.set_ft_sensor_zero()
                if return_code(result) == 0 and arm.error_code == 0:
                    time.sleep(0.5)
                    print(
                        f'FT sensor enabled and zeroed at {robot_ip} '
                        f'(attempt {attempt}/2)')
                    return 0
                last_failure = (
                    f'zero returned {result}, controller error='
                    f'{arm.error_code}')
            print(
                f'FT sensor recovery attempt {attempt}/2 failed: '
                f'{last_failure}', file=sys.stderr)
            time.sleep(0.5)
        print(
            f'FT sensor initialization failed after two attempts: '
            f'{last_failure}. Check sensor wiring and power.',
            file=sys.stderr)
        return 1
    finally:
        arm.disconnect()


if __name__ == '__main__':
    raise SystemExit(main())
