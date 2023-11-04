def hash_fnv1a_32(data: bytes):
    hash = 0x811c9dc5
    for x in data:
        hash = ((ord(x) ^ hash) * 0x01000193) & 0xFFFFFFFF
    return hash