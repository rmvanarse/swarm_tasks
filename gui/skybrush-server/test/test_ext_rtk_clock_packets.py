from datetime import datetime, timezone
from struct import pack

from flockwave.gps.ubx.enums import UBXClass, UBXNAVSubclass
from flockwave.gps.ubx.packet import UBXPacket
from pytest import approx

from flockwave.server.ext.rtk.clock_packets import GPSClockPacketParser


def test_nav_timeutc_packet_returns_precise_unix_timestamp() -> None:
    parser = GPSClockPacketParser()

    payload = pack(
        "<IIiHBBBBBB",
        0,
        0,
        250_000_000,
        2024,
        1,
        2,
        3,
        4,
        5,
        0x04,
    )
    packet = UBXPacket(UBXClass.NAV, UBXNAVSubclass.TIMEUTC, payload)

    timestamp = parser.parse(packet)

    expected = datetime(2024, 1, 2, 3, 4, 5, tzinfo=timezone.utc).timestamp() + 0.25
    assert timestamp == approx(expected)


def test_nav_timeutc_packet_without_valid_utc_time_is_ignored() -> None:
    parser = GPSClockPacketParser()

    payload = pack(
        "<IIiHBBBBBB",
        0,
        0,
        250_000_000,
        2024,
        1,
        2,
        3,
        4,
        5,
        0x00,
    )
    packet = UBXPacket(UBXClass.NAV, UBXNAVSubclass.TIMEUTC, payload)

    assert parser.parse(packet) is None
