from roboat_perception.dtp import hash_fnv1a_32
from bitstring import Bits

def test_fnv1a_32_calc():
    b1 = Bits(b'abc123')
    # Use bitstring here
    assert hash_fnv1a_32(b1) == Bits('0x38b29a05')

# def test_change_to_bits():
#     pass