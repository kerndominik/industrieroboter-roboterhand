#!/usr/bin/env python3
"""Serial <-> TCP bridge (e.g. to reach a COM port of Windows from WSL 2).

WSL 2 does not see USB serial devices without usbipd-win. As an alternative,
run this script on Windows and let robothand_interface connect over TCP:

    Windows:  py -3 tools\serial_tcp_bridge.py COM5            (pip install pyserial)
    WSL:      ros2 launch robothand_bringup robothand.launch.py port:=socket://localhost:5555

With WSL "networkingMode=mirrored" localhost works directly. In NAT mode use
the Windows host IP (see /etc/resolv.conf) and --host 0.0.0.0.

The serial port stays open while clients come and go, so an Arduino does not
reset on every reconnect. Only one client is served at a time.
"""

import argparse
import socket
import sys
import threading

import serial


def pump_serial_to_socket(ser, conn, stop):
    while not stop.is_set():
        try:
            data = ser.read(ser.in_waiting or 1)
            if data:
                conn.sendall(data)
        except (OSError, serial.SerialException):
            stop.set()


def pump_socket_to_serial(ser, conn, stop):
    while not stop.is_set():
        try:
            data = conn.recv(4096)
        except OSError:
            data = b''
        if not data:
            stop.set()
            break
        try:
            ser.write(data)
        except serial.SerialException:
            stop.set()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('port', help='serial port, e.g. COM5 or /dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--host', default='127.0.0.1', help='listen address')
    parser.add_argument('--tcp-port', type=int, default=5555)
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.host, args.tcp_port))
    server.listen(1)
    print(f'bridging {args.port} @ {args.baud} baud <-> tcp://{args.host}:{args.tcp_port}', flush=True)
    try:
        while True:
            conn, addr = server.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f'client {addr[0]}:{addr[1]} connected', flush=True)
            ser.reset_input_buffer()
            stop = threading.Event()
            threads = [threading.Thread(target=pump_serial_to_socket, args=(ser, conn, stop), daemon=True),
                       threading.Thread(target=pump_socket_to_serial, args=(ser, conn, stop), daemon=True)]
            for t in threads:
                t.start()
            stop.wait()
            conn.close()
            for t in threads:
                t.join(timeout=1.0)
            print('client disconnected', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        server.close()
        ser.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
