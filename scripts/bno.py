import time

import adafruit_bno055
import board

def fmt(v, w=8, p=2):
    if v is None:
        return ' ' * w
    return f'{v:{w}.{p}f}'


def fmt_int(v):
    return ' ' * 8 if v is None else f'{v:^8d}'


def main():
    i2c = board.I2C()
    sensor = adafruit_bno055.BNO055_I2C(i2c)  # add address=0x29 if needed

    # Hide cursor (optional, looks nicer)
    print('\033[?25l', end='')

    try:
        # Print static layout once
        print('BNO055 Live Data')
        print('-' * 60)
        print('Euler (deg):       X        Y        Z')
        print('Quaternion:        W        X        Y        Z')
        print('Calibration:       SYS      GYR      ACC      MAG')
        print('-' * 60)

        while True:
            # Move cursor up 3 lines to overwrite dynamic data only
            print('\033[3A', end='')

            e = sensor.euler or (None, None, None)
            q = sensor.quaternion or (None, None, None, None)
            c = sensor.calibration_status or (None, None, None, None)

            # Euler
            print(
                'Euler (deg):   '
                f"{fmt(e[0])} {fmt(e[1])} {fmt(e[2])}      "
            )

            # Quaternion
            print(
                'Quaternion:    '
                f"{fmt(q[0])} {fmt(q[1])} {fmt(q[2])} {fmt(q[3])}"
            )

            # Calibration (0–3 values)
            print(
                'Calibration:   '
                f"{fmt_int(c[0])}{fmt_int(c[1])}{fmt_int(c[2])}{fmt_int(c[3])}"
            )

            time.sleep(0.1)
    finally:
        print('\033[?25h', end='')


if __name__ == '__main__':
    main()
