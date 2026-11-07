"""Configuration schema for the swarm_bridge extension."""

schema = {
    "properties": {
        "command_host": {
            "type": "string",
            "title": "Command UDP host",
            "description": "Address main.py's UDP command receiver listens on.",
            "default": "127.0.0.1",
        },
        "command_port": {
            "type": "number",
            "title": "Command UDP port",
            "default": 12008,
        },
        "id_map": {
            "type": "object",
            "title": "UAV ID map",
            "description": (
                "Maps the UAV id Skybrush shows (from the MAVLink system id) "
                "to the id main.py uses for the same drone. Ids that are not "
                "listed are passed through unchanged."
            ),
            "default": {},
        },
        "terrain_warnings": {
            "type": "boolean",
            "title": "Warn about terrain",
            "description": (
                "After a goal, search or split is accepted, check its route or "
                "area against the terrain extension's data and warn in Live if "
                "drones would fly too close to the ground. Needs the terrain "
                "extension and main.py's config panel (telemetry_port)."
            ),
            "default": True,
        },
        "telemetry_port": {
            "type": "number",
            "title": "main.py config panel port",
            "description": (
                "Port of main.py's config panel on command_host. Its telemetry "
                "tells the server the height each drone is set to fly at."
            ),
            "default": 8765,
        },
        "default_guided_circle_direction": {
            "type": "number",
            "title": "Default guided-circle direction",
            "default": 1,
        },
        "default_guided_circle_radius": {
            "type": "number",
            "title": "Default guided-circle radius (m)",
            "default": 50,
        },
    },
    "required": [],
}
