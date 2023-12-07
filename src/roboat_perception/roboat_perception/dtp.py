from bitstring import Bits, BitArray
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
    check_sum = hash_fnv1a_32(data)
    # adding packet num
    store.prepend(BitArray(1))
    # adding hashed data to be sent
    store.append(check_sum[0:16])
    # output is full message
    return store
    # calculate size

# takes bits, checks proper size, checks if the hash is accurate
def decode_packet(data: BitArray) -> Bits:
    # Store packet number to find length of message
    packet_num = data[0]
    # Store part of message that should be compared to
    checker = data[-16:]
    # Store message
    if packet_num:
        data2 = data[1:-16]
    else:
        data2 = data[:-16]
    # Store part of message to hash & compare with
    checksum = hash_fnv1a_32(data2[8:])
    # Compare
    if checksum[0:16] != checker:
        return None
    # Returning message
    return data2[8:]
