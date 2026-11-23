"""Virtual serial device that behaves like the robot hand firmware.

Creates a pseudo terminal (Linux) and runs the firmware emulation on it, so the
complete serial code path of robothand_interface can be tested without
hardware:

    ros2 run robothand_interface fake_mcu                # -> /tmp/ttyROBOTHAND
    ros2 run robothand_interface robothand_interface --ros-args -p port:=/tmp/ttyROBOTHAND
"""

import argparse
import os
import select
import time
import tty

from .links import EmulatedMcu


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--link', default='/tmp/ttyROBOTHAND',
                        help='symlink that points to the pseudo terminal')
    parser.add_argument('--rate', type=float, default=50.0, help='telemetry rate [Hz]')
    args, _ = parser.parse_known_args(argv)  # ignore --ros-args

    master, slave = os.openpty()
    tty.setraw(slave)
    slave_name = os.ttyname(slave)
    if os.path.islink(args.link):
        os.unlink(args.link)
    os.symlink(slave_name, args.link)
    print(f'fake robot hand MCU on {slave_name} (symlink {args.link})', flush=True)

    mcu = EmulatedMcu(telemetry_hz=args.rate)
    mcu.open()
    rx = b''
    try:
        while True:
            readable, _, _ = select.select([master], [], [], 0.005)
            if readable:
                rx += os.read(master, 1024)
                *lines, rx = rx.split(b'\n')
                for line in lines:
                    mcu.write_line(line.decode('ascii', errors='replace'))
            out = mcu.read_lines()
            if out:
                try:
                    os.write(master, ('\n'.join(out) + '\n').encode('ascii'))
                except OSError:
                    time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        if os.path.islink(args.link):
            os.unlink(args.link)
        os.close(master)
        os.close(slave)


if __name__ == '__main__':
    main()
