"""Extension that serves terrain (ground elevation) data: a shaded-relief map
overlay for Skybrush Live, height-above-ground lookups, and terrain data for
the drones."""

from .extension import dependencies, description, exports, run, schema

__all__ = ("dependencies", "description", "exports", "run", "schema")
