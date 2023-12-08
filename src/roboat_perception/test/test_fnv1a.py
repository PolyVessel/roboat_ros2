from roboat_perception.dtp import DTP
from bitstring import Bits

def test_fnv1a_32_calc():
    dtp = DTP(Bits(b'abc123'))
    # Use bitstring here
    assert dtp.hash_fnv1a_32(dtp.data) == Bits('0x38b29a05')
