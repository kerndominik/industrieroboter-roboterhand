"""Communication links to the robot hand microcontroller.

All links offer the same small interface::

    open()  close()  is_open  write_line(str)  read_lines() -> list[str]

* SerialLink   - real USB serial port (pyserial). The port may also be a
                 pyserial URL, e.g. ``socket://localhost:5555`` (see
                 tools/serial_tcp_bridge.py) or ``rfc2217://host:port``.
* EmulatedMcu  - a Python model of the firmware (same protocol, servo ramps,
                 fake strain gauge signals) for running without hardware.
"""

import math
import random
import time
from typing import List, Optional

from . import protocol


class LinkError(Exception):
    pass


class SerialLink:
    """Non-blocking, line based serial link."""

    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self._serial = None
        self._rx = bytearray()

    @property
    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def open(self) -> None:
        import serial  # imported lazily so the simulation works without pyserial
        try:
            ser = serial.serial_for_url(self.port, do_not_open=True)
            ser.baudrate = self.baudrate
            ser.timeout = 0
            ser.write_timeout = 0.5
            # Keep DTR/RTS low: avoids holding an ESP32 in reset / boot mode.
            # (Arduino Mega still resets once when the port is opened.)
            if not self.port.startswith(('socket://', 'rfc2217://', 'loop://')):
                ser.dtr = False
                ser.rts = False
            ser.open()
            self._serial = ser
            self._rx.clear()
        except (serial.SerialException, OSError, ValueError) as exc:
            self._serial = None
            raise LinkError(f'cannot open {self.port}: {exc}') from exc

    def close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:  # noqa: BLE001 - closing must never raise
                pass
        self._serial = None

    def write_line(self, line: str) -> None:
        if not self.is_open:
            raise LinkError('port not open')
        try:
            self._serial.write((line + '\n').encode('ascii'))
        except Exception as exc:  # serial.SerialException, OSError, timeout
            self.close()
            raise LinkError(f'write failed: {exc}') from exc

    def read_lines(self) -> List[str]:
        if not self.is_open:
            return []
        try:
            data = self._serial.read(4096)
        except Exception as exc:  # serial.SerialException, OSError
            self.close()
            raise LinkError(f'read failed: {exc}') from exc
        if not data:
            return []
        self._rx.extend(data)
        if len(self._rx) > 16384:  # garbage without newlines - drop it
            self._rx.clear()
            return []
        *lines, rest = self._rx.split(b'\n')
        self._rx = bytearray(rest)
        return [ln.decode('ascii', errors='replace').strip('\r') for ln in lines]


class EmulatedMcu:
    """Software model of the robot hand firmware (for simulation and tests)."""

    def __init__(self, n_servos: int = protocol.NUM_SERVOS, n_gauges: int = 5,
                 home_deg: float = 10.0, limits_deg=(0.0, 180.0),
                 telemetry_hz: float = 50.0, max_speed_deg_s: float = 180.0,
                 servo_open_deg: Optional[List[float]] = None,
                 servo_closed_deg: Optional[List[float]] = None,
                 clock=time.monotonic):
        self.n_servos = n_servos
        self.n_gauges = n_gauges
        self.limits = limits_deg
        self.home = [home_deg] * n_servos
        self.target = list(self.home)
        self.current = list(self.home)
        self.enabled = False
        self.max_speed = max_speed_deg_s
        self.telemetry_period = 1.0 / telemetry_hz if telemetry_hz > 0 else 0.0
        # only used for the fake strain gauge signal (contact when closing)
        self.servo_open = servo_open_deg or [10.0] * n_servos
        self.servo_closed = servo_closed_deg or [170.0] * n_servos
        self.gauge_offset = [0.0] * n_gauges
        self._clock = clock
        self._t0 = clock()
        self._last_update = self._t0
        self._last_telemetry = self._t0
        self._out: List[str] = []
        self._open = False

    # -- link interface -----------------------------------------------------
    @property
    def is_open(self) -> bool:
        return self._open

    def open(self) -> None:
        self._open = True
        self._out.append(self._info())
        self._out.append('# ready (emulated)')

    def close(self) -> None:
        self._open = False

    def write_line(self, line: str) -> None:
        if not self._open:
            raise LinkError('port not open')
        self._update()
        self._handle(line.strip())

    def read_lines(self) -> List[str]:
        if not self._open:
            return []
        self._update()
        now = self._clock()
        if self.telemetry_period > 0 and now - self._last_telemetry >= self.telemetry_period:
            # fixed rate without drift (catch up at most one period)
            self._last_telemetry = max(self._last_telemetry + self.telemetry_period,
                                       now - self.telemetry_period)
            self._out.extend(self._telemetry())
        out, self._out = self._out, []
        return out

    # -- firmware model -----------------------------------------------------
    def _ms(self) -> int:
        return int((self._clock() - self._t0) * 1000)

    def _info(self) -> str:
        return f'I ROBOTHAND EMULATED 1.0.0 {self.n_servos} {self.n_gauges} SIM'

    def _clamp(self, deg: float) -> float:
        return min(max(deg, self.limits[0]), self.limits[1])

    def _update(self) -> None:
        now = self._clock()
        dt = now - self._last_update
        self._last_update = now
        step = self.max_speed * dt
        for i in range(self.n_servos):
            err = self.target[i] - self.current[i]
            if self.max_speed <= 0 or abs(err) <= step:
                self.current[i] = self.target[i]
            else:
                self.current[i] += math.copysign(step, err)

    def _gauge_raw(self) -> List[float]:
        values = []
        for i in range(self.n_gauges):
            closure = 0.0
            if i < self.n_servos:
                span = self.servo_closed[i] - self.servo_open[i]
                if span != 0:
                    closure = (self.current[i] - self.servo_open[i]) / span
            # "object contact" above 60 % closure -> force rises
            contact = max(0.0, closure - 0.6) / 0.4
            raw = 50.0 + 900.0 * contact + random.gauss(0.0, 4.0)
            values.append(raw)
        return values

    def _telemetry(self) -> List[str]:
        t = self._ms()
        lines = [f'A {t} {1 if self.enabled else 0} ' + ' '.join(f'{a:.1f}' for a in self.current)]
        if self.n_gauges > 0:
            raw = self._gauge_raw()
            lines.append(f'G {t} ' + ' '.join(
                str(int(round(r - o))) for r, o in zip(raw, self.gauge_offset)))
        return lines

    def _handle(self, line: str) -> None:
        if not line:
            return
        cmd = line[0].upper()
        try:
            args = [float(a) for a in line[1:].replace(',', ' ').split()]
        except ValueError:
            self._out.append('ERR syntax')
            return
        if cmd == 'S':
            if len(args) != self.n_servos:
                self._out.append('ERR S needs one angle per servo')
                return
            self.target = [self._clamp(a) for a in args]
            self.enabled = True
        elif cmd == 'P':
            if len(args) != 2 or not 0 <= int(args[0]) < self.n_servos:
                self._out.append('ERR servo index')
                return
            self.target[int(args[0])] = self._clamp(args[1])
            self.enabled = True
        elif cmd == 'E' and len(args) == 1:
            self.enabled = args[0] != 0
            self._out.append('OK E')
        elif cmd == 'V' and len(args) == 1 and args[0] >= 0:
            self.max_speed = args[0]
            self._out.append('OK V')
        elif cmd == 'R' and len(args) == 1 and 0 <= args[0] <= 200:
            self.telemetry_period = 1.0 / args[0] if args[0] > 0 else 0.0
            self._out.append('OK R')
        elif cmd == 'H':
            self.target = [self._clamp(a) for a in self.home]
            self.enabled = True
            self._out.append('OK H')
        elif cmd == 'T':
            self.gauge_offset = self._gauge_raw()
            self._out.append('OK T')
        elif cmd == 'Q':
            self._out.extend(self._telemetry())
        elif cmd == 'I':
            self._out.append(self._info())
        else:
            self._out.append('ERR unknown command')
