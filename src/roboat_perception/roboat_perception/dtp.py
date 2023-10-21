# e5d52855

def djb2_calc(dataL: bytes) -> bytes:
    hash = 5381
    for b in dataL:
        hash = (((hash << 5) + hash) + b) & 0xFFFFFFFF
    return hash