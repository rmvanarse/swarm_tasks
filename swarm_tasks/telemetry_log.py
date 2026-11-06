"""Persistent per-run log for the swarm's guidance loop.

One plain-text file per run (~/Documents/swarm_env/logs/<run_id>.log),
appended to on every reposition cycle for every bot (same cadence as
the real MAVLink DO_REPOSITION command -- see swarm.py's
_guide_and_reposition) and on every discrete mission event (goal
advanced, final waypoint captured, loiter started, self-heal). Always
on, independent of Variables.debug_reposition -- that flag only
controls whether the SAME information also prints to the console.

The point: when something looks wrong live (e.g. "it gets stuck
between waypoint 1 and 2") and the cause isn't obvious from watching
the plot, this file has the full history of what every bot was
actually doing, tick by tick -- not just whatever's still in console
scrollback. Ask to check it and the log path/contents can be read
back directly instead of guessing from code alone.
"""

import logging
import os

LOG_DIR = os.path.join(os.path.expanduser("~"), "Documents", "swarm_env", "logs")


def setup_swarm_logger(run_id):
    os.makedirs(LOG_DIR, exist_ok=True)
    logger = logging.getLogger("swarm")
    logger.setLevel(logging.INFO)
    logger.propagate = False
    if logger.handlers:
        # Already set up -- e.g. a second Swarm constructed in the same
        # process. Keep the first run's file rather than opening a
        # second handler that would duplicate every line.
        return logger
    path = os.path.join(LOG_DIR, f"{run_id}.log")
    handler = logging.FileHandler(path, encoding="utf-8")
    handler.setFormatter(
        logging.Formatter("%(asctime)s.%(msecs)03d %(message)s", datefmt="%Y-%m-%d %H:%M:%S")
    )
    logger.addHandler(handler)
    print(f"Swarm log: {path}")
    return logger
