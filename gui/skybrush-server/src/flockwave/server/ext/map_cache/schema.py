"""Configuration schema for the map_cache extension."""

schema = {
    "properties": {
        "mapbox_access_token": {
            "type": "string",
            "title": "Mapbox access token",
            "description": (
                "Token the server adds to Mapbox tile requests. Leave empty to "
                "use the MAPBOX_ACCESS_TOKEN environment variable, or the key "
                "set in Live's API keys settings. Prefer the environment "
                "variable so that the token does not end up in a config file."
            ),
            "default": "",
        },
        "max_size_mb": {
            "type": "number",
            "title": "Maximum cache size (MB)",
            "description": "Oldest tiles are deleted when the cache grows past this. 0 means unlimited.",
            "default": 2048,
        },
        "max_age_days": {
            "type": "number",
            "title": "Refresh tiles older than (days)",
            "description": (
                "Older tiles are re-downloaded when the server is online. "
                "When it is offline the old tile is served anyway."
            ),
            "default": 30,
        },
        "allowed_hosts": {
            "type": "array",
            "title": "Allowed tile hosts",
            "description": (
                "Host name patterns (wildcards allowed) the server may fetch "
                "tiles from. Anything else is refused. Empty means the built-in "
                "list of the map providers Live supports."
            ),
            "items": {"type": "string"},
            "default": [],
        },
    },
    "required": [],
}
