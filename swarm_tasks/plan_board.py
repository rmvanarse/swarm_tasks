"""The search plan, kept so a map (Skybrush Live) can show what each drone is
going to do: the area, each drone's strip of it, its back-and-forth search
lines and the path it will really fly.

Everything here is plain data in latitude/longitude. The swarm's worker threads
write it and the HTTP server thread reads it, so every access takes a lock.
"""

import itertools
import math
import threading

#: A path is thinned to about this many points; a map does not need more.
MAX_PATH_POINTS = 400


def square_ring(lat, lon, size_m):
    """Corners (south-west first, counter-clockwise, closed) of a square of
    `size_m` metres centred on (lat, lon)."""
    half = size_m / 2.0
    dlat = half / 111_320.0
    dlon = half / (111_320.0 * max(math.cos(math.radians(lat)), 0.01))
    ring = [
        [lat - dlat, lon - dlon],
        [lat - dlat, lon + dlon],
        [lat + dlat, lon + dlon],
        [lat + dlat, lon - dlon],
    ]
    ring.append(list(ring[0]))
    return [[round(a, 7), round(b, 7)] for a, b in ring]


def thin(points, limit=MAX_PATH_POINTS):
    """At most about `limit` of `points`, evenly spread, keeping the last."""
    points = list(points)
    if len(points) <= limit:
        return points
    step = math.ceil(len(points) / limit)
    kept = points[::step]
    if kept[-1] != points[-1]:
        kept.append(points[-1])
    return kept


class PlanBoard:
    """Jobs that are planned or running, newest last."""

    def __init__(self):
        self._lock = threading.Lock()
        self._jobs = []
        self._ids = itertools.count(1)
        self._version = 0

    def publish(self, kind, label, areas, bots):
        """Adds a job.

        kind: "search", "split", ...; label: text for the map.
        areas: [{"center": [lat, lon], "size_m": metres, "ids": [bot ids],
                 "ring": optional outline, else the square around the centre}]
        bots: [{"bot": bot id, "sys_id": system id, "area": index into areas,
                "strip": {...}, "sweeps": [...], "path": [[lat, lon], ...]}]

        A bot has one plan at a time: it is taken out of any earlier job, and
        a job left with no bots disappears.
        """
        job_areas = [
            {
                "ring": a.get("ring")
                or square_ring(a["center"][0], a["center"][1], a["size_m"]),
                "center": [float(a["center"][0]), float(a["center"][1])],
                "size_m": float(a["size_m"]),
                "bots": sorted(int(i) for i in a.get("ids", [])),
            }
            for a in areas
        ]
        job_bots = [
            {**b, "path": thin(b.get("path", [])), "status": "active"} for b in bots
        ]
        with self._lock:
            self._drop_locked({b["bot"] for b in job_bots})
            job_id = next(self._ids)
            self._jobs.append(
                {
                    "id": job_id,
                    "kind": kind,
                    "label": label,
                    "areas": job_areas,
                    "bots": job_bots,
                }
            )
            self._version += 1
            return job_id

    def mark_bots(self, bot_ids, status):
        """Sets "status" ("active", "left" or "removed") of these bots. A bot
        that left its job keeps its strip on the map: the area is still being
        finished, by whoever took its work over."""
        ids = set(bot_ids)
        with self._lock:
            changed = False
            for job in self._jobs:
                for b in job["bots"]:
                    if b["bot"] in ids and b.get("status") != status:
                        b["status"] = status
                        changed = True
            if changed:
                self._version += 1

    def drop_bots(self, bot_ids):
        """Forgets the plan of these bots (they were given other work, were
        removed from the swarm, ...)."""
        with self._lock:
            if self._drop_locked(set(bot_ids)):
                self._version += 1

    def clear(self):
        with self._lock:
            if self._jobs:
                self._jobs = []
                self._version += 1

    def snapshot(self):
        """JSON-ready copy: {"version": n, "jobs": [...]}. The version changes
        whenever the plan does, so a poller can skip an unchanged one."""
        with self._lock:
            return {
                "version": self._version,
                "jobs": [
                    {**job, "bots": [dict(b) for b in job["bots"]]}
                    for job in self._jobs
                ],
            }

    def _drop_locked(self, bot_ids):
        changed = False
        kept = []
        for job in self._jobs:
            bots = [b for b in job["bots"] if b["bot"] not in bot_ids]
            if len(bots) != len(job["bots"]):
                changed = True
            if not bots:
                continue
            job = {**job, "bots": bots}
            job["areas"] = [
                {**a, "bots": [i for i in a["bots"] if i not in bot_ids]}
                for a in job["areas"]
            ]
            # an area nobody works on any more is not drawn
            job["areas"] = [a for a in job["areas"] if a["bots"]]
            kept.append(job)
        self._jobs = kept
        return changed
