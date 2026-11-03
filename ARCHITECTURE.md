# Swarm GCS — architecture & run guide

This covers the GUI/GCS layer built on top of the `swarm_tasks` library
(see `README.md` for that). Three programs take part:

- **`main.py`** (swarm code) — holds its own MAVLink/dronekit connection to
  every drone, runs the guidance/kinematics logic (`swarm.py`), and listens
  for swarm commands over UDP.
- **`skybrush-server`** — the GCS back end. Its standard `mavlink` extension
  receives each drone's telemetry; this project's `swarm_bridge` extension
  forwards swarm operations to `main.py`.
- **`skybrush-live`** — the operator web GUI. It only talks to the server.

## Two independent data paths per drone

Each drone sends **two separate MAVLink streams**:

- **Stream A** (`serial1`, UDP port 14550) goes to the server's `mavlink`
  extension. This is what Live shows: position, battery, flight mode, and
  the stock arm / takeoff / loiter / land / RTH controls.
- **Stream B** (`serial2` / `serial4`, UDP ports 14551, 14552, ...) goes to
  `main.py`. The swarm code uses it to fly goals, searches and splits.

```mermaid
flowchart LR
    D["Drones (1..N)"]
    M["main.py<br/>swarm code<br/>UDP :12008 commands"]
    subgraph S["skybrush-server"]
        MV["mavlink extension<br/>listens UDP :14550"]
        SB["swarm_bridge extension<br/>command sender only"]
    end
    L["skybrush-live<br/>operator GUI"]

    D -- "stream A: udp 14550" --> MV
    D -- "stream B: udp 1455X" --> M
    MV -- "UAV-INF telemetry" --> L
    L -- "stock commands<br/>arm, takeoff, loiter, land" --> MV
    L -- "X-SWARM-CMD<br/>goal, search, split, stop" --> SB
    SB -- "UDP :12008" --> M
```

- **Telemetry**: drone → server `mavlink` extension → Live. `main.py` is not
  involved, so Live keeps working if `main.py` is down.
- **Stock vehicle commands** (arm, disarm, takeoff, loiter, land, RTH): Live →
  server `mavlink` extension → drone.
- **Swarm commands**: Live sends one `X-SWARM-CMD` message → `swarm_bridge`
  builds the exact text string `main.py` expects → one UDP datagram to
  `command_host:command_port`. Fire-and-forget; `main.py` never replies.

## UAV ids

Skybrush names a UAV from its MAVLink system id. `main.py` uses its own ids
(`1..N`, in connection order). When they differ, set `id_map` in
`swarm.jsonc` (Skybrush id → `main.py` id). Ids you do not list pass through
unchanged. The map is applied to the ids inside `goal`, `split`,
`specificsplit`, `remove` and `add`.

## Takeoff altitude and loiter

- **Takeoff** opens a prompt for the altitude in meters above home (default
  10, remembered between uses). Live sends it as `altitude` in `UAV-TAKEOFF`.
  Without a value the server uses 2.5 m
  (`DEFAULT_TAKEOFF_ALTITUDE_M` in `ext/mavlink/driver.py`).
- **Hold** now sets **Loiter** mode (`loiter`: ArduCopter mode 5, ArduPlane
  mode 12) instead of Position Hold. On a quadplane this is the fixed-wing
  loiter, which circles.

## The swarm command set

Every command below is handled by `main.py`'s dispatch loop and sent over UDP
port `12008` as one of these strings (see `send_command_example.py` for a
reference client, and `ext/swarm_bridge/driver.py` for the server's builder):

| Command | Wire format | What it does |
|---|---|---|
| `goal` | `goal_<json [[lat,lon],...]>_<direction>_<radius>_<json ids>` | Sends the listed bot ids to fly a waypoint path |
| `stop` | `stop` | Global stop of whatever is currently running |
| `search` | `search,<lat>,<lon>,<num_uavs>,<grid_space>,<coverage_area>` | Area-coverage search with every active bot |
| `split` | `split_<centers json>_<ids json>_<grid_space json>_<coverage_area json>` | Auto-splits coverage areas across the given bots |
| `specificsplit` | `specificsplit_<centers json>_<uav_array json>_<grid_space json>_<coverage_area json>` | Like `split`, but you pick which bots cover which area |
| `remove` | `remove_<json [id]>` | Self-heal: freeze a bot, hand its goals to the next free bot |
| `add` | `add_<json [id]>` | Bring a removed bot back |
| `different` | `different,<start>,<diff>` | Staggers every bot's altitude (`start + i*diff` meters) |

## Running it

### 1. Start the drones

Each drone must send stream A to the machine running `skybrush-server` on
UDP 14550 and stream B to `main.py`'s address on UDP 1455X. With the SITL
launcher this is automatic. Close any other ground-station app (for example
Mission Planner) that holds UDP port 14550.

### 2. Start `main.py`

```bat
run_main.bat <bind_ip> <num_bots>
```

For example `run_main.bat 172.24.96.1 3` for three SITL drones in WSL. It
prints `Ip address: <ip>`; that is the `command_host` for step 3.
`SWARM_IP` and `SWARM_NUM_BOTS` are the environment variables behind the two
arguments.

### 3. Configure and start the server

Edit `gui/skybrush-server/etc/conf/swarm.jsonc`:

```jsonc
"mavlink": { "enabled": true, "networks": { "mav": { "connections": ["default"] } } },
"swarm_bridge": {
  "enabled": true,
  "command_host": "<the IP main.py printed>",
  "command_port": 12008,
  "id_map": {}            // e.g. { "11": "1", "12": "2" } when ids differ
}
```

Then run `gui\skybrush-server\run_server.bat`. The log should show
`Connection at :14550 up and running` and
`swarm_bridge: commands go to udp://<host>:12008`.

### 4. Start Live

```bash
cd gui/live
npm install   # first time only
npm start
```

Open the URL it prints and connect to `localhost:5000` (the server runs on
the same machine as Live).

### 5. Fly

Drones appear in the UAVs list from their MAVLink heartbeats. Use the UAV
toolbar for takeoff, loiter, land and RTH, and the Swarm ops dialog for
goal, search and split.

## Troubleshooting

- **No drones in Live**: check the server log for `Connection at :14550` and
  `mav/<id>` lines. If missing, another program is holding UDP 14550, or the
  drones are not sending stream A to this machine.
- **Swarm commands do nothing**: look for `swarm_bridge: sent ...` in the
  server log. If present, check that `main.py` is running, that `command_host`
  is the address it bound to, and that no firewall blocks UDP 12008.
- **Commands act on the wrong drone**: the Skybrush and `main.py` ids differ;
  fill in `id_map`.
- **Only some drones connected to `main.py`**: its startup line
  `Number of Drones Connected: X/Y` shows failed connections; bot ids follow
  connection order.

## Drone-show code

The drone light-show features were removed. The server's `mavlink` extension
still imports a few show types, so small no-op stubs remain in
`ext/show/`, `show/` and `ext/mavlink/_show_stubs.py`. They do nothing.

## Key files

| File | Role |
|---|---|
| `main.py` | Swarm code entry point: `Vehicles`/`Variables`/`Swarm` and the UDP command loop |
| `run_main.bat` | Starts `main.py` with a bind IP and drone count |
| `swarm.py`, `vehicle.py` | Guidance logic and dronekit connections |
| `swarm_tasks/network/UDP.py` | UDP command receiver (`main.py` side) |
| `gui/skybrush-server/src/flockwave/server/ext/swarm_bridge/` | Handles `X-SWARM-CMD` and sends the UDP commands |
| `gui/skybrush-server/etc/conf/swarm.jsonc` | Server config: `mavlink`, `command_host`, `id_map` |
| `gui/skybrush-server/run_server.bat` | Starts the server with that config |
| `gui/live/src/features/swarm-ops/` | Live dialog and `X-SWARM-CMD` actions |
