"""Generic parsing helpers for the flight-area / geofence / safety sub-objects
that appear inside mission item parameters (``UPDATE_FLIGHT_AREA``,
``UPDATE_GEOFENCE``, ``UPDATE_SAFETY`` in :mod:`flockwave.server.model.mission`).

These were originally written against the drone-show specification format
(because that format happens to define the same ``flightArea`` / ``geofence``
/ ``safety`` / ``coordinateSystem`` sub-schemas), but nothing about the
parsing itself is show-specific, so this module carries them without the
show-upload framing now that the show extension has been removed.
"""

from collections.abc import Sequence

from flockwave.gps.vectors import (
    FlatEarthCoordinate,
    FlatEarthToGPSCoordinateTransformation,
    GPSCoordinate,
)

from flockwave.server.utils import optional_float

from .flight_area import FlightAreaConfigurationRequest, FlightAreaPolygon
from .geofence import GeofenceAction, GeofenceConfigurationRequest, GeofencePolygon
from .safety import LowBatteryThreshold, LowBatteryThresholdType, SafetyConfigurationRequest

__all__ = (
    "get_coordinate_system_from_mission_params",
    "is_coordinate_system_in_mission_params_geodetic",
    "get_flight_area_configuration_from_mission_params",
    "get_geofence_configuration_from_mission_params",
    "get_safety_configuration_from_mission_params",
)


def get_coordinate_system_from_mission_params(
    params: dict,
) -> FlatEarthToGPSCoordinateTransformation:
    """Returns the local-to-global coordinate system described by the
    ``coordinateSystem`` key of a mission item's parameters.
    """
    coordinate_system = params.get("coordinateSystem")
    try:
        return FlatEarthToGPSCoordinateTransformation.from_json(coordinate_system)
    except Exception:
        raise RuntimeError(
            "Invalid or missing coordinate system specification"
        ) from None


def is_coordinate_system_in_mission_params_geodetic(params: dict) -> bool:
    """Checks whether the ``coordinateSystem`` key of a mission item's
    parameters denotes plain geodetic (lat/lon) coordinates.
    """
    coordinate_system = params.get("coordinateSystem")
    return isinstance(coordinate_system, str) and coordinate_system == "geodetic"


def _parse_points(
    points: Sequence[list[int | float]],
    coordinate_system: FlatEarthToGPSCoordinateTransformation | None,
) -> list[GPSCoordinate]:
    return [
        (
            GPSCoordinate.from_json(point[:2])  # [lat, lon] in [deg1e-7]
            if coordinate_system is None
            else coordinate_system.to_gps(FlatEarthCoordinate(point[0], point[1], 0))
        )  # [x, y] local coordinates as float
        for point in points
    ]


def _parse_flight_area_polygon(
    polygon: dict, coordinate_system: FlatEarthToGPSCoordinateTransformation | None
) -> FlightAreaPolygon:
    is_inclusion = bool(polygon.get("isInclusion", False))
    points = polygon.get("points", ())

    if points and points[0] == points[-1]:
        points.pop()

    return FlightAreaPolygon(_parse_points(points, coordinate_system), is_inclusion=is_inclusion)


def _parse_geofence_polygon(
    polygon: dict, coordinate_system: FlatEarthToGPSCoordinateTransformation | None
) -> GeofencePolygon:
    is_inclusion = bool(polygon.get("isInclusion", False))
    points = polygon.get("points", ())

    if points and points[0] == points[-1]:
        points.pop()

    return GeofencePolygon(_parse_points(points, coordinate_system), is_inclusion=is_inclusion)


def get_flight_area_configuration_from_mission_params(
    params: dict,
) -> FlightAreaConfigurationRequest:
    result = FlightAreaConfigurationRequest()

    flight_area = params.get("flightArea", None)
    if not flight_area:
        return result

    version = flight_area.get("version", 0)
    if version is None:
        raise RuntimeError("flight area specification must have a version number")
    if version != 1:
        raise RuntimeError("only version 1 flight areas are supported")

    result.max_altitude = optional_float(flight_area.get("maxAltitude"))
    result.min_altitude = optional_float(flight_area.get("minAltitude"))

    polygons = flight_area.get("polygons", ())
    if polygons:
        coordinate_system = (
            None
            if is_coordinate_system_in_mission_params_geodetic(params)
            else get_coordinate_system_from_mission_params(params)
        )
        result.polygons = [
            _parse_flight_area_polygon(polygon, coordinate_system) for polygon in polygons
        ]

    return result


def get_geofence_configuration_from_mission_params(
    params: dict,
) -> GeofenceConfigurationRequest:
    result = GeofenceConfigurationRequest()

    geofence = params.get("geofence", None)
    if not geofence:
        return result

    version = geofence.get("version", 0)
    if version is None:
        raise RuntimeError("geofence specification must have a version number")
    if version != 1:
        raise RuntimeError("only version 1 geofences are supported")

    result.enabled = bool(geofence.get("enabled", True))
    result.max_altitude = optional_float(geofence.get("maxAltitude"))
    result.max_distance = optional_float(geofence.get("maxDistance"))
    result.min_altitude = optional_float(geofence.get("minAltitude"))

    action = geofence.get("action")
    if action:
        try:
            result.action = GeofenceAction(action)
        except ValueError:
            raise RuntimeError(f"unknown geofence action: {action!r}") from None

    polygons = geofence.get("polygons", ())
    rally_points = geofence.get("rallyPoints", ())

    if polygons or rally_points:
        coordinate_system = (
            None
            if is_coordinate_system_in_mission_params_geodetic(params)
            else get_coordinate_system_from_mission_params(params)
        )

        if polygons:
            result.polygons = [
                _parse_geofence_polygon(polygon, coordinate_system) for polygon in polygons
            ]

        if rally_points:
            result.rally_points = _parse_points(rally_points, coordinate_system)

    return result


def get_safety_configuration_from_mission_params(
    params: dict,
) -> SafetyConfigurationRequest:
    result = SafetyConfigurationRequest()

    safety = params.get("safety", None)
    if not safety:
        return result

    version = safety.get("version", 0)
    if version is None:
        raise RuntimeError("safety specification must have a version number")
    if version not in (1, 2):
        raise RuntimeError("only version 1 or 2 safety specifications are supported")

    if version == 1:
        voltage = optional_float(safety.get("lowBatteryVoltage"))
        result.low_battery_threshold = (
            LowBatteryThreshold(type=LowBatteryThresholdType.VOLTAGE, value=voltage)
            if voltage is not None
            else None
        )
    else:
        threshold = safety.get("lowBatteryThreshold")
        result.low_battery_threshold = (
            LowBatteryThreshold.from_json(threshold) if threshold is not None else None
        )

    result.critical_battery_voltage = optional_float(safety.get("criticalBatteryVoltage"))
    result.return_to_home_altitude = optional_float(safety.get("returnToHomeAltitude"))
    result.return_to_home_speed = optional_float(safety.get("returnToHomeSpeed"))

    return result
