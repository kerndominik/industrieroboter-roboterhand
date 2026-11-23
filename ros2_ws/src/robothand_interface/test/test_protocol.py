from robothand_interface import protocol as p


def test_parse_info():
    msg = p.parse_line('I ROBOTHAND MEGA2560 1.0.0 5 5 ANALOG\r\n')
    assert isinstance(msg, p.InfoMessage)
    assert (msg.board, msg.firmware, msg.n_servos, msg.n_gauges, msg.gauge_source) == \
        ('MEGA2560', '1.0.0', 5, 5, 'ANALOG')
    assert msg.description == 'ROBOTHAND MEGA2560 1.0.0'


def test_parse_servo_status():
    msg = p.parse_line('A 1234 1 10.0 20.5 30.0 40.0 50.0')
    assert isinstance(msg, p.ServoStatus)
    assert msg.t_ms == 1234 and msg.enabled
    assert msg.angles_deg == [10.0, 20.5, 30.0, 40.0, 50.0]


def test_parse_gauges_negative():
    msg = p.parse_line('G 20 457 -14 0 8 -393')
    assert isinstance(msg, p.GaugeStatus)
    assert msg.values == [457, -14, 0, 8, -393]


def test_parse_ack_error_log():
    assert p.parse_line('OK V') == p.Ack('V')
    assert p.parse_line('ERR unknown command') == p.ErrorMessage('unknown command')
    assert p.parse_line('# ready') == p.LogMessage('ready')


def test_parse_garbage_is_ignored():
    for line in ['', '   ', 'ets Jun  8 2016 00:22:57', 'A x y', 'G 1 2 abc', '\x00\xff']:
        assert p.parse_line(line) is None


def test_commands():
    assert p.cmd_set_all([10, 20.25, 30, 40, 170]) == 'S 10.0 20.2 30.0 40.0 170.0'
    assert p.cmd_set_one(2, 90) == 'P 2 90.0'
    assert p.cmd_enable(True) == 'E 1' and p.cmd_enable(False) == 'E 0'
    assert p.cmd_speed(-5) == 'V 0.0'
    assert p.cmd_rate(50) == 'R 50.0'
