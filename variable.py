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
        # chase_sep_m below for the other direction (bot too far
        # ahead), which is handled by slowing down instead.
        self.resync_sep_m = 100.0
        # Separation (metres) beyond which the BOT (not the drone) is
        # judged too far ahead -- swarm.py ramps the bot's speed down
        # from its normal cruise at this distance to a floor at
        # resync_sep_m, so the real drone gets a chance to close the
        # gap and both keep moving together/simultaneously, instead of
        # the bot continuing to pull away on its own until a hard
        # resync is the only option. Must stay below resync_sep_m.
        self.chase_sep_m = 40.0
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

        self.tick_interval_s = 0.1
        self.run_id = f"{time.strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:4]}"
        self.heights = []

        # Minimum time between send_reposition() MAVLink calls per bot --
        # the tick loop runs every tick_interval_s (much faster), so this
        # keeps the actual command rate to the real vehicle throttled
        # independently of how fast the sim ticks.
        self.reposition_interval_s = 0.4
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
