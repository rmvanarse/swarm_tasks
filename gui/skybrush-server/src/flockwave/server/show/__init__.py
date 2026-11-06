"""Stubs for the removed drone-show helpers still imported by the mavlink and
virtual_uavs drivers. Show upload is not supported in this fork."""

from typing import Any

ShowSpecification = dict


def _unsupported(*args: Any, **kwds: Any):
    raise NotImplementedError("drone shows are not supported in this fork")


get_altitude_reference_from_show_specification = _unsupported
get_coordinate_system_from_show_specification = _unsupported
get_geofence_configuration_from_show_specification = _unsupported
get_light_program_from_show_specification = _unsupported
