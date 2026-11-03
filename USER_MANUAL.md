# Swarm GCS — operator user manual

This is the day-to-day "how do I actually fly the swarm from the GUI" guide.
For how the pieces talk to each other, see [ARCHITECTURE.md](ARCHITECTURE.md).

## Before you start

Each drone sends two MAVLink streams: one to the server (what Live shows)
and one to `main.py` (the swarm code). Start things in this order:

1. **Drones** (or the SITL launcher). Close other ground-station apps such as
   Mission Planner, because they hold UDP port 14550.
2. **Swarm code**: `run_main.bat <bind_ip> <num_bots>`, for example
   `run_main.bat 172.24.96.1 3`. Note the `Ip address: <ip>` line it prints.
3. **Server**: set `command_host` in `swarm.jsonc` to that IP (and `id_map` if
   Skybrush ids differ from `main.py` ids), then run
   `gui\skybrush-server\run_server.bat`.
4. **Live**: `cd gui/live && npm start`, then open the URL it prints in a
   browser (the plain browser build, not an Electron build).

Connect Skybrush Live to the server: open the server-connection dialog and
connect to `localhost:5000`.

Drones appear in the UAV list within a few seconds, from their MAVLink
heartbeats, with live position, battery and flight mode.

## The UAV toolbar

Above the UAV list panel. Stock vehicle buttons use the server's MAVLink link
directly:

- **Takeoff** asks for the altitude (meters above home) first, then arms and
  takes off the selected drones.
- **Loiter** (flag icon) switches the selected drones to Loiter mode.
- **Return to home**, **Land**, **Arm**, **Disarm** work as usual.

The swarm buttons, from left to right:

| Button | What it does | Needs a selection? |
|---|---|---|
| Stop (red stop icon) | Stops whatever the swarm is currently running (search/goal/split) | No |
| Remove from swarm | Self-heal: freezes the selected bot(s), hands their remaining work to another bot | Yes |
| Add back to swarm | Brings a previously-removed bot back into service | Yes |
| Swarm ops (hub icon) | Opens the dialog for Goal/Search/Split/Specific split/Heights | No (selection matters per-tab, see below) |

Select UAVs by clicking them in the list or on the map (shift/ctrl-click for
multiple).

## Sending Goal, Search, Split, or Specific split

These four all need something drawn on the map — a waypoint path or
center points — instead of typing coordinates by hand. It's a
two-step flow every time:

**Step 1 — draw the shape.** Use the drawing toolbar on the **left edge of
the map** (close the Swarm ops dialog first if it's open — it's a modal and
blocks map clicks while open):

| Tool (left toolbar) | Draws | Used by |
|---|---|---|
| Add marker | A single point | Search's center; Split and Specific split (one point per sub-area) |
| Draw path | An ordered sequence of waypoints | Goal |

Click the tool, then click on the map to place each point. For a path,
double-click (or press Enter) to finish the shape. It stays
visible on the map as a named feature once drawn.

**Step 2 — open Swarm ops and pick it.** Click the hub icon in the UAV
toolbar, choose the matching tab:

- **Goal** — select the target UAV(s) first, pick the drawn path from the
  dropdown, set guided-circle direction/radius, **Send goal**.
- **Search** — pick the drawn center point, set grid spacing and coverage
  area, **Start search**. Targets the whole swarm automatically; UAV
  selection is ignored.
- **Split** — select the target UAV(s), tick the drawn points to use (one
  sub-area around each point), set grid spacing/coverage area, **Split**.
- **Specific split** — tick the drawn points, type bot-id groups as
  `1,2;3,4` (bots 1+2 to the first point, 3+4 to the second, in drawn
  order), set grid spacing/coverage area, **Specific split**.
- **Heights** — no drawing needed: start height + step between bots,
  **Apply heights**. Sets every bot's altitude at once.

If you haven't drawn the right shape yet, the tab shows a plain message
telling you which tool to use instead of an empty/broken dropdown, and the
action button stays disabled until something valid is picked.

## After you send a command

Goal and Split show a confirmation dialog first (they're consequential and
selection-based); Stop/Search/Specific split/Heights fire immediately. Every
command shows a snackbar afterward summarizing success/failure counts.

**Important limitation**: `main.py`'s UDP protocol has no acknowledgment.
"Success" in that snackbar means *the command was sent*, not that `main.py`
confirmed it actually executed — if the swarm-master machine is unreachable
or not running, you likely won't be told.

## Troubleshooting

- **No drones appear at all**: check the server log for
  `Connection at :14550 up and running` and `mav/<id>` lines. If they are
  missing, another program (such as Mission Planner) is holding UDP port
  14550, or the drones are not sending their first stream to this machine.
- **Swarm commands do nothing**: look for `swarm_bridge: sent ...` in the
  server log, then confirm `main.py` is running (it prints every command it
  receives) and that nothing blocks UDP port `12008`.
- **A command moves the wrong drone**: the UAV ids in Live differ from
  `main.py`'s ids. Fill in `id_map` in `swarm.jsonc`.
- **A Goal/Search/Split/Specific split tab won't let you submit**: you
  haven't drawn the matching shape yet (see the table above) — draw it with
  the dialog closed, then reopen Swarm ops.
- **Stop/Search/Specific split/Heights seem to do nothing**: these target
  the whole swarm regardless of selection, so UAV selection isn't the
  issue — check the `swarm_bridge:` log lines and confirm `main.py` is
  receiving the command (it prints each one).
