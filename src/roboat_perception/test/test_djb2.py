from roboat_perception.dtp import djb2_calc

def test_djb2_calc():
    b1 = b'abc123'
    assert djb2_calc(b1) == b'\xf1\x47\xeb\xc1'