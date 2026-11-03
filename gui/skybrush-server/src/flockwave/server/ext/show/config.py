"""Stub types used by the mavlink extension (see ext/show/__init__.py)."""

from enum import Enum


class AuthorizationScope(Enum):
    NONE = "none"
    LIVE = "live"
    REHEARSAL = "rehearsal"
    LIGHTS_ONLY = "lights"


class DroneShowConfiguration:
    """Placeholder; this fork never builds one."""
