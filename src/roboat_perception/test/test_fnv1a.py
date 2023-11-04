from roboat_perception.dtp import hash_fnv1a_32

def test_fnv1a_32_calc():
    b1 = b'abc123'
    # Use bitstring here
    assert hash_fnv1a_32(b1) == '38b29a05' 