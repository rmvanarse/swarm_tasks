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
