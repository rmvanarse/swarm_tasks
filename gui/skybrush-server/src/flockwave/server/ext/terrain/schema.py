"""Configuration schema for the terrain extension."""

schema = {
    "properties": {
        "source_url": {
            "type": "string",
            "title": "Elevation tile URL",
            "description": (
                "Where missing SRTM tiles are downloaded from. {name} is the "
                "tile name (N12E080) and {ns_lat} its folder (N12). Empty means "
                "the built-in public source."
            ),
            "default": "",
        },
        "offline": {
            "type": "boolean",
            "title": "Never download",
            "description": (
                "Use only the tiles already on disk. Use the pre-download "
                "request while online to prepare an area."
            ),
            "default": False,
        },
        "data_dirs": {
            "type": "array",
            "title": "Folders with elevation tiles",
            "description": (
                "Extra folders with ready-made SRTM .hgt tiles (for example "
                "tiles downloaded from terrain.ardupilot.org/SRTM1/). They are "
                "read first, never changed, and work with no internet."
            ),
            "items": {"type": "string"},
            "default": [],
        },
        "prefetch_areas": {
            "type": "array",
            "title": "Areas to download at start",
            "description": (
                "Circles ({\"lat\": .., \"lon\": .., \"radius_km\": ..}) whose "
                "elevation tiles are downloaded in the background when the "
                "server starts and is online, so they are available offline "
                "later. Tiles already on disk are not downloaded again."
            ),
            "items": {"type": "object"},
            "default": [],
        },
        "min_clearance_m": {
            "type": "number",
            "title": "Minimum height above ground (m)",
            "description": "Below this a warning is shown in Live.",
            "default": 30,
        },
        "lookahead_s": {
            "type": "number",
            "title": "Look-ahead time (s)",
            "description": (
                "The warning also considers where a drone will be this many "
                "seconds from now, if it keeps its current velocity."
            ),
            "default": 10,
        },
    },
    "required": [],
}
