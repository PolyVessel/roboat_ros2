from roboat_perception.dtp import DecodedPacket
from roboat_perception.dtp import hash_fnv1a_32
from roboat_perception.dtp import create_packet
from roboat_perception.dtp import decode_packet
from bitstring import Bits, BitArray
import unittest

class TestDTP(unittest.TestCase):
    def test_fnv1a_32_calc(self):
        assert hash_fnv1a_32(Bits(b'abc123')) == Bits('0x38b29a05')

    def test_create_packet(self):
        data = Bits(b'abc123')
        store = create_packet(data)
        expected = BitArray('0x0001000161626331323338b2')
        self.assertEqual(store, expected)

    def test_decode_packet_valid(self):
        data = Bits(b'abc123')
        store = create_packet(data)
        output = decode_packet(store)
        self.assertEqual(output, DecodedPacket(Bits(b'abc123'), BitArray('0x0001'), BitArray('0x0001')))
    
    def test_decode_packet_invalid(self):
        data = Bits(b'ac123')
        encoded = create_packet(data)
        del encoded[36:44]
        output = decode_packet(encoded)
        self.assertIsNone(output)
        
if __name__ == '__main__':
    unittest.main()