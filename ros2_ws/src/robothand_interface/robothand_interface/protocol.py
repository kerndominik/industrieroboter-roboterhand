"""Serial protocol between the PC and the robot hand microcontroller.

Line based ASCII protocol, 115200 baud, every line terminated with '\\n'.
See embedded_software/*/<sketch>.ino for the firmware side.

PC -> board::

    S <a0> .. <a4>     target angles of all servos [deg]
    P <i> <a>          target angle of servo i [deg]
    E <0|1>            disable / enable servos
    V <deg_per_s>      max. servo speed (0 = no ramp)
    R <hz>             telemetry rate (0 = off)
    H                  home position
    T                  tare strain gauges
    Q                  request one telemetry frame
    I                  request board info

board -> PC::

    I ROBOTHAND <board> <fw> <n_servos> <n_gauges> <sg_source>
    A <t_ms> <enabled> <c0> .. <c4>
    G <t_ms> <g0> .. <gN>
    OK <cmd>
    ERR <message>
    # <log text>
"""

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Union

FINGERS = ('thumb', 'index', 'middle', 'ring', 'little')
NUM_SERVOS = len(FINGERS)


@dataclass
class InfoMessage:
    board: str
    firmware: str
    n_servos: int
    n_gauges: int
    gauge_source: str

    @property
    def description(self) -> str:
        return f'ROBOTHAND {self.board} {self.firmware}'


@dataclass
class ServoStatus:
    t_ms: int
    enabled: bool
    angles_deg: List[float] = field(default_factory=list)


@dataclass
class GaugeStatus:
    t_ms: int
    values: List[float] = field(default_factory=list)


@dataclass
class Ack:
    command: str


@dataclass
class ErrorMessage:
    message: str


@dataclass
class LogMessage:
    text: str


Message = Union[InfoMessage, ServoStatus, GaugeStatus, Ack, ErrorMessage, LogMessage]


def parse_line(line: str) -> Optional[Message]:
    """Parse one line received from the board.

    Returns None for empty or malformed lines (e.g. boot garbage of the ESP32).
    """
    line = line.strip()
    if not line:
        return None
    if line.startswith('#'):
        return LogMessage(line[1:].strip())
    parts = line.split()
    kind = parts[0]
    try:
        if kind == 'A' and len(parts) >= 3:
            return ServoStatus(int(parts[1]), parts[2] == '1', [float(p) for p in parts[3:]])
        if kind == 'G' and len(parts) >= 2:
            return GaugeStatus(int(parts[1]), [float(p) for p in parts[2:]])
        if kind == 'I' and len(parts) >= 7 and parts[1] == 'ROBOTHAND':
            return InfoMessage(parts[2], parts[3], int(parts[4]), int(parts[5]), parts[6])
        if kind == 'OK' and len(parts) == 2:
            return Ack(parts[1])
        if kind == 'ERR':
            return ErrorMessage(line[3:].strip())
    except ValueError:
        return None
    return None


def _fmt(value: float) -> str:
    return f'{value:.1f}'


def cmd_set_all(angles_deg: Sequence[float]) -> str:
    return 'S ' + ' '.join(_fmt(a) for a in angles_deg)


def cmd_set_one(index: int, angle_deg: float) -> str:
    return f'P {int(index)} {_fmt(angle_deg)}'


def cmd_enable(enable: bool) -> str:
    return 'E 1' if enable else 'E 0'


def cmd_speed(deg_per_s: float) -> str:
    return f'V {_fmt(max(0.0, deg_per_s))}'


def cmd_rate(hz: float) -> str:
    return f'R {_fmt(max(0.0, hz))}'


def cmd_home() -> str:
    return 'H'


def cmd_tare() -> str:
    return 'T'


def cmd_query() -> str:
    return 'Q'


def cmd_info() -> str:
    return 'I'
