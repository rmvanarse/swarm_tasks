"""Class that validates whether the clock of the GPS is in sync with the
clock of the computer running the server.
"""

from datetime import datetime, timezone
from struct import Struct
from typing import ClassVar

from flockwave.gps.ubx.enums import UBXClass, UBXNAVSubclass
from flockwave.gps.ubx.packet import UBXPacket

from .types import GPSPacket


class GPSClockPacketParser:
    """Class that attempts to parse UTC timestamp from incoming GPS packets."""

    _ubx_nav_timeutc_struct: ClassVar[Struct] = Struct("<8xiHBBBBB")

    def parse(self, packet: GPSPacket) -> float | None:
        """Notifies the clock synchronization validator about the arrival of a
        new packet from the GPS and tries to parse the UTC timestamp from it.

        Args:
            packet: the incoming GPS packet

        Returns:
            the parsed UTC timestamp as a UNIX timestamp in seconds if the
            packet contained a valid U-blox NAV-TIMEUTC message; ``None``
            otherwise
        """
        if (
            isinstance(packet, UBXPacket)
            and packet.class_id == UBXClass.NAV
            and packet.subclass_id == UBXNAVSubclass.TIMEUTC
        ):
            return self._handle_ubx_nav_timeutc(packet)

        return None

    def _handle_ubx_nav_timeutc(self, packet: UBXPacket) -> float | None:
        """Handles a NAV-TIMEUTC packet.

        Parses the UTC datetime from the packet, converts it into a UNIX
        timestamp in seconds, and checks whether it matches the current
        date/time on the computer running the server.

        Returns:
            the parsed UTC timestamp as a UNIX timestamp in seconds if parsing
            was successful; ``None`` otherwise
        """
        payload = packet.payload
        if len(payload) < 20:
            # Invalid or short packet
            return

        if payload[19] & 0x04 != 0x04:
            # Packet does not contain a valid UTC timestamp
            return

        try:
            struct = self._ubx_nav_timeutc_struct
            nanosecond, year, month, day, hour, minute, second = struct.unpack(
                payload[: struct.size]
            )
            dt = datetime(year, month, day, hour, minute, second, tzinfo=timezone.utc)
            unix_timestamp: float = dt.timestamp() + nanosecond / 1_000_000_000
        except Exception:
            return None

        return unix_timestamp
