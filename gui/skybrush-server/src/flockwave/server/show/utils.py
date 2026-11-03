"""Stub for the removed show utilities; only the MAVFTP checksum is kept."""

from zlib import crc32 as _crc32


def crc32_mavftp(data: bytes, value: int = 0) -> int:
    return _crc32(data, value) & 0xFFFFFFFF
