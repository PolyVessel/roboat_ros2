from bitstring import Bits, BitArray
from dataclasses import dataclass
@dataclass
class DTP:
    data: Bits
    
    @staticmethod
    def hash_fnv1a_32(data: Bits) -> Bits:
        # pad w/ 0s after subtracting length of smaller 
        hash = 0x811c9dc5
        for x in data.cut(8):
            hash = (((hash ^ x.uintbe) * 0x01000193)) % (2 ** 32)
            
        return Bits(uintbe=hash, length=32)

    # packet num & total packet num is 1,
    # create bits object for pck num, 16 bits
    # add data, add checksum
    def create_packet(data: Bits) -> BitArray:
        store = BitArray(data)
        check_sum = self.hash_fnv1a_32(data)
        # adding packet num
        store.prepend(BitArray(1))
        # adding hashed data to be sent
        store.append(check_sum[0:16])
        # output is full message
        return store
        # calculate size

    # takes bits, checks proper size, checks if the hash is accurate
    def decode_packet(data: BitArray) -> Bits:
        # Store part of message to compare
        checker = data[-16:]
        # Store message
        data2 = data[32:-16]
        # Store part of message to hash & compare with
        checksum = self.hash_fnv1a_32(data2)
        # Compare
        if checksum[0:16] != checker:
            return None
        # Returning message
        self.data = Bits(data[16:32] + data2)
        return self.data
