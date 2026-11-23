from robothand_interface import protocol as p
from robothand_interface.links import EmulatedMcu


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def messages(mcu):
    return [p.parse_line(line) for line in mcu.read_lines()]


def test_boot_and_info():
    clk = FakeClock()
    mcu = EmulatedMcu(clock=clk)
    mcu.open()
    msgs = messages(mcu)
    assert isinstance(msgs[0], p.InfoMessage)
    assert msgs[0].n_servos == 5


def test_ramp_and_telemetry():
    clk = FakeClock()
    mcu = EmulatedMcu(clock=clk, max_speed_deg_s=100.0, telemetry_hz=10.0)
    mcu.open()
    mcu.read_lines()
    mcu.write_line('S 110 10 10 10 10')
    clk.t = 0.5
    msgs = [m for m in messages(mcu) if isinstance(m, p.ServoStatus)]
    assert msgs and msgs[-1].enabled
    assert abs(msgs[-1].angles_deg[0] - 60.0) < 1e-6     # 10 + 100 deg/s * 0.5 s
    clk.t = 2.0
    msgs = [m for m in messages(mcu) if isinstance(m, p.ServoStatus)]
    assert msgs[-1].angles_deg[0] == 110.0


def test_errors_and_limits():
    clk = FakeClock()
    mcu = EmulatedMcu(clock=clk, max_speed_deg_s=0.0)
    mcu.open()
    mcu.read_lines()
    mcu.write_line('S 1 2')
    mcu.write_line('X')
    mcu.write_line('S 500 -20 90 90 90')
    mcu.write_line('Q')
    msgs = messages(mcu)
    errors = [m for m in msgs if isinstance(m, p.ErrorMessage)]
    status = [m for m in msgs if isinstance(m, p.ServoStatus)]
    assert len(errors) == 2
    assert status[-1].angles_deg[:2] == [180.0, 0.0]


def test_tare_zeroes_gauges():
    clk = FakeClock()
    mcu = EmulatedMcu(clock=clk)
    mcu.open()
    mcu.write_line('T')
    mcu.write_line('Q')
    gauges = [m for m in messages(mcu) if isinstance(m, p.GaugeStatus)]
    assert all(abs(v) < 40 for v in gauges[-1].values)
