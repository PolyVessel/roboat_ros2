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
def create_packet(data: Bits) -> Bits:
    result = BitArray('data')
    result.append()
    return result
    # calculate size

check = create_packet(b'abc123')
print(check)
# 
# def change_to_bits(data: Bits) -> Bits:
#     # Get the hashing output
#     store = hash_fnv1a_32(data)
#     # Convert to bits
#     txt = Bits(store)
#     output = txt.__str__

# .