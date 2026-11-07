"""What a mission planner knew about one drone's share of the search area, kept
so the plan can be shown on a map: the strip it was given and the back-and-forth
lines it will sweep (all latitude/longitude)."""

from geopy.distance import distance


def strip_record(bottom, top, west_edge, full_width, sweeps):
    """bottom/top: geopy points on the strip's south and north edge; west_edge:
    point on its west edge; full_width: strip width in metres; sweeps: list of
    ((lat, lon), (lat, lon)) lines, one per sweep."""
    east = distance(meters=full_width).destination(west_edge, 90)
    return {
        "south": float(bottom.latitude),
        "north": float(top.latitude),
        "west": float(west_edge.longitude),
        "east": float(east.longitude),
        "sweeps": [
            [[float(a[0]), float(a[1])], [float(b[0]), float(b[1])]]
            for a, b in sweeps
        ],
    }
