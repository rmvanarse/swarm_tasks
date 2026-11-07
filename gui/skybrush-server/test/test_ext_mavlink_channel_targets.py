"""TERRAIN_DATA has no target fields; messages that do have them keep them."""

from flockwave.protocols.mavlink.introspection import import_dialect

from flockwave.server.ext.mavlink.channel import (
    _create_mavlink_message,
    encode_mavlink_message_from_spec,
)


def make_link():
    dialect = import_dialect("ardupilotmega")
    return dialect.MAVLink(None, srcSystem=254, srcComponent=190)


def test_target_fields_are_left_out_of_messages_that_do_not_have_them():
    link = make_link()
    spec = (
        "TERRAIN_DATA",
        {
            "lat": 125000000,
            "lon": 805000000,
            "grid_spacing": 30,
            "gridbit": 3,
            "data": list(range(16)),
            # what UAV-targeted sending always adds
            "target_system": 1,
            "target_component": 1,
            "_mavlink_version": 2,
        },
    )
    packet = encode_mavlink_message_from_spec(spec, link)
    assert isinstance(packet, bytes) and len(packet) > 0

    decoded = link.decode(bytearray(packet))
    assert decoded.get_type() == "TERRAIN_DATA"
    assert decoded.gridbit == 3 and list(decoded.data) == list(range(16))
    assert decoded.grid_spacing == 30


def test_messages_with_targets_still_get_them():
    link = make_link()
    message = _create_mavlink_message(
        link,
        "COMMAND_LONG",
        target_system=7,
        target_component=1,
        command=400,
        confirmation=0,
        param1=1,
        param2=0,
        param3=0,
        param4=0,
        param5=0,
        param6=0,
        param7=0,
    )
    assert message.target_system == 7 and message.target_component == 1
