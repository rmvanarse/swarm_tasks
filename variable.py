import json
import os
import time
import uuid
from utils import read_origin

# Same folder as rectangles.yaml (see utils.read_origin) -- one place for
# this project's runtime config. Whatever the config-server web panel
# saves here is re-applied on top of the defaults below the next time a
# Variables() is constructed, so edits survive a restart.
DEFAULT_CONFIG_PATH = os.path.join(
    os.path.expanduser("~"), "Documents", "swarm_env", "variables_config.json"
)


def _coerce(current, raw):
    """Parse `raw` into whatever type `current` already is, so a field
    never silently changes type out from under the code reading it."""
    if isinstance(current, bool):
        if isinstance(raw, bool):
            return raw
        return str(raw).strip().lower() in ("1", "true", "yes", "on")
    if isinstance(current, int):
        return int(float(raw))
    if isinstance(current, float):
        return float(raw)
    if isinstance(current, (list, tuple)):
        values = raw if isinstance(raw, (list, tuple)) else json.loads(raw)
        # Match the element type too, or an int tuple like world_size
        # silently becomes a float one.
        element = type(current[0]) if current else float
        cast = int if element is int else float
        return type(current)(cast(float(v)) for v in values)
    return raw


class Variables:
    def __init__(
        self,
        world_size=(3000, 3000),
        speed=30,
        copter_speed=5.0,
        bank_angle_deg=30,
        vehicle_loiter_radius_min_m=1100.0,
        config_path=DEFAULT_CONFIG_PATH,
    ):
        self._config_path = config_path

        self.origin = read_origin()
        self.world_size = world_size

        self.speed = speed  # fixedwing/quadplane bot speed
        self.copter_speed = copter_speed  # copter/ground bot speed

        self.lead_speed_boost = 5.0
        # Hard-resync distance (metres): once bot<->drone separation
        # exceeds this AND the drone has moved ahead of the bot
        # (lead_state), swarm.py gives up trying to guide the bot back
        # gradually and just teleports it onto the drone's real
        # position -- the gap's too big to close smoothly. See
        # the *_chase_sep_m settings below for the other direction (bot too far
        # ahead), which is handled by slowing down instead.
        self.resync_sep_m = 100.0
        # The BOT (not the drone) slows down when it gets too far ahead, so
        # both keep moving together. Per vehicle type: beyond *_chase_sep_m
        # the bot's speed ramps down, and at *_hold_sep_m it is at its floor
        # (a copter can stop; a fixedwing keeps half its speed). Keep
        # chase < hold <= resync_sep_m. The copter leash is tight because
        # a copter lags right after a goal change (it must brake and turn).
        self.copter_chase_sep_m = 10.0
        self.copter_hold_sep_m = 30.0
        self.fixedwing_chase_sep_m = 60.0
        self.fixedwing_hold_sep_m = 100.0

        # Copter acceleration limit for the bot, so it cannot outrun the real
        # drone. The limit is read from each drone's WPNAV_ACCEL; this scale
        # multiplies it (1.0 = match the drone, 0 = no limit). Used when a
        # drone did not report WPNAV_ACCEL. Both are read on every tick.
        self.copter_accel_scale = 1.0
        self.copter_accel_fallback_mps2 = 2.5
        self.bank_angle_deg = bank_angle_deg
        # Lookahead distance (metres) ahead of a FIXEDWING bot that
        # compute_lookahead_target() projects the real MAVLink
        # DO_REPOSITION target from -- main.py derives the base value
        # from Loiter_param() (the largest WP_LOITER_RAD among
        # connected fixedwing drones), which is None whenever no
        # connected drone reported one (an all-copter swarm, or one
        # whose planes hadn't finished downloading parameters yet) --
        # treat that as "nothing to go on" rather than crashing on
        # None + 150.
        self.vehicle_loiter_radius_min_m = (vehicle_loiter_radius_min_m or 0) + 150
        # Same role as vehicle_loiter_radius_min_m above, but for
        # copter/ground bots -- there's no copter-equivalent of
        # WP_LOITER_RAD to auto-detect a base value from (a copter
        # doesn't orbit, it hovers), and a fixedwing-scale lookahead
        # (hundreds of metres) is wildly too far ahead for a copter's
        # much slower, more precise flight -- without this, every
        # copter bot used the SAME huge fixedwing lookahead, sending
        # the real copter toward a point it could take minutes to
        # reach even though its bot was already there.
        self.copter_lookahead_m = 20.0

        # How close a bot must get to a waypoint to count as having reached
        # it, per vehicle type. Read on every tick, so an edit in the config
        # panel applies to the running mission straight away. The values are
        # fixed here, not taken from the drone's own parameters.
        self.fixedwing_capture_radius_m = 50.0
        self.copter_capture_radius_m = 2.0

        # Minimum distance a bot keeps from the other bots, per vehicle type.
        # Also read on every tick. Keep it clearly below the spacing between
        # neighbouring routes, or a bot can end up unable to reach its goal.
        self.fixedwing_separation_m = 100.0
        self.copter_separation_m = 20.0

        self.tick_interval_s = 0.1
        self.run_id = f"{time.strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:4]}"
        self.heights = []

        # Minimum time between send_reposition() MAVLink calls per bot --
        # the tick loop runs every tick_interval_s (much faster), so this
        # keeps the actual command rate to the real vehicle throttled
        # independently of how fast the sim ticks.
        self.reposition_interval_s = 0.4

        # Self-heal hand-over: the work a bot leaves behind is cut into
        # connected pieces of about this many metres, and each piece goes to
        # the bot expected to finish earliest (see Swarm._hand_over).
        self.handover_chunk_m = 150.0
        # Print what each bot is actually commanding, once per bot every
        # 2s: the lat/lon sent, where the drone reports itself, and how
        # far the bot has run ahead of it. Set False to silence.
        self.debug_reposition = True

        self.endDistance = 50000

        # This runs as a headless backend service -- no plot window by
        # default. Flip to True (via the config panel or the saved
        # config file) only for local debugging with a display attached.
        self.gui = False
        # Anything saved from a previous run (e.g. edited through the
        # config-server web panel) overrides the defaults set above.
        self.load()

    def _public_fields(self):
        return [k for k in vars(self) if not k.startswith("_")]

    def load(self, path=None):
        """Apply values saved at `path` on top of the current values.
        A missing file or unreadable JSON is not an error -- it just
        means nothing has been saved yet."""
        path = path or self._config_path
        try:
            with open(path) as f:
                saved = json.load(f)
        except (OSError, ValueError):
            return
        known = set(self._public_fields())
        changed = set()
        for name, raw in saved.items():
            if name not in known:
                continue
            try:
                setattr(self, name, _coerce(getattr(self, name), raw))
            except (TypeError, ValueError):
                continue
            changed.add(name)

    def save(self, fields=None, path=None):
        """Persist field values to `path` (default: the path this instance
        loads from), so edits made at runtime survive a restart.

        `fields=None` (the default) snapshots every public field. Pass an
        explicit iterable of names -- as the config-server panel does, with
        the fields it just applied -- to instead MERGE just those values
        onto whatever is already saved. That distinction matters: a few
        fields (origin, vehicle_loiter_radius_min_m) are meant to be
        recomputed fresh at every startup (read from rectangles.yaml / the
        live vehicle) unless a user has explicitly overridden them. A
        blanket snapshot on every save would freeze them at whatever they
        happened to be when an unrelated field was edited, silently
        breaking that "fresh every run" behaviour.
        """
        path = path or self._config_path
        os.makedirs(os.path.dirname(path), exist_ok=True)

        if fields is None:
            names = self._public_fields()
            data = {}
        else:
            try:
                with open(path) as f:
                    data = json.load(f)
            except (OSError, ValueError):
                data = {}
            names = fields

        for name in names:
            value = getattr(self, name)
            data[name] = list(value) if isinstance(value, tuple) else value

        # Write to a temp file then rename, so a crash mid-write can't
        # leave behind a truncated/corrupt config file.
        tmp_path = path + ".tmp"
        with open(tmp_path, "w") as f:
            json.dump(data, f, indent=2, sort_keys=True)
        os.replace(tmp_path, path)
