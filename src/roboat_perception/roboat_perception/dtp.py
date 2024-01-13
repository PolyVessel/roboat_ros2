from bitstring import Bits, BitArray
from dataclasses import dataclass

@dataclass
class DecodedPacket:
    data: Bits
    packet_num: Bits
    total_packet_num: Bits
    
def hash_fnv1a_32(data: Bits) -> Bits:
    # pad w/ 0s after subtracting length of smaller 
    hash = 0x811c9dc5
    for x in data.cut(8):
        hash = (((hash ^ x.uintbe) * 0x01000193)) % (2 ** 32)
    return Bits(uintbe=hash, length=32)

# packet num & total packet num is 1, create bits object for pck num, 16 bits
# add data, add checksum
def create_packet(data: Bits) -> BitArray:
    store = BitArray(data)
    check_sum = hash_fnv1a_32(data)
    
    # Total Pkt Num is 1
    store.prepend(BitArray(uint=1, length=16))
    
    # adding packet num
    store.prepend(BitArray(uint=1, length=16))
    
    # adding hashed data to be sent
    store.append(check_sum[0:16])
    # outputting the full message
    return store

# takes bits, checks proper size, checks if the hash is accurate
def decode_packet(packet: BitArray) ->  DecodedPacket | None:
    # Store part of message to compare
    checker = packet[-16:]
    # Store message
    message = packet[32:-16]
    # Store part of message to hash & compare with
    checksum = hash_fnv1a_32(message)
    # Compare
    if checksum[0:16] != checker:
        return None
    # Returning message
    output = DecodedPacket(Bits(message), packet[0:16], packet[16:32])
    return output
