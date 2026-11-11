"""
CRC-16/CCITT-FALSE implementation.
Poly: 0x1021, Init: 0xFFFF, RefIn: False, RefOut: False, XorOut: 0x0000.
"""

# Precomputed 256-entry lookup table for CRC-16/CCITT-FALSE
CRC16_TABLE = []
for i in range(256):
    curr = (i << 8) & 0xFFFF
    for _ in range(8):
        if curr & 0x8000:
            curr = ((curr << 1) ^ 0x1021) & 0xFFFF
        else:
            curr = (curr << 1) & 0xFFFF
    CRC16_TABLE.append(curr)


def crc16_ccitt_false(data: bytes, initial: int = 0xFFFF) -> int:
    """Computes CRC-16/CCITT-FALSE over data using precomputed lookup table."""
    crc = initial
    for byte in data:
        crc = ((crc << 8) & 0xFFFF) ^ CRC16_TABLE[(crc >> 8) ^ byte]
    return crc
