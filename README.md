# Swarm GCS (hybrid)

Ground control for a **mixed swarm**: quadcopters and fixed-wing / quadplane
(VTOL) drones flying together, commanded from a web GUI.

It has three programs:

| Program | Where | Role |
|---|---|---|
| `main.py` (+ `swarm.py`, `vehicle.py`) | repo root | Swarm master. Connects to every drone with dronekit, plans and flies goals, searches and area splits, listens for commands on UDP `12008`. |
| `skybrush-server` | `gui/skybrush-server` | GCS back end (Python / Trio). Its `mavlink` extension feeds telemetry to the GUI; this repo adds the `swarm_bridge` and `map_cache` extensions. |
| `skybrush-live` | `gui/live` | Operator web GUI (React / TypeScript). Only talks to the server. |

Details and diagrams: **[ARCHITECTURE.md](ARCHITECTURE.md)**. Flying from the
GUI: **[USER_MANUAL.md](USER_MANUAL.md)**. Jetson install:
**[JETSON_SETUP.md](JETSON_SETUP.md)**.

## How it fits together

Each drone sends **two MAVLink streams**:

- **Stream A** (UDP `14550`) goes to the server's `mavlink` extension. Live shows
  this: position, battery, flight mode, and the stock controls (arm, takeoff,
  loiter, guided, land, return to home).
- **Stream B** (UDP `14551`, `14552`, ...) goes to `main.py`, which flies the
  swarm with it.

Swarm commands (goal, search, split, stop, remove/add, heights) go
Live -> server `swarm_bridge` -> one UDP datagram to `main.py` on port `12008`.
Telemetry never passes through `main.py`, so Live keeps working if it is down.

## Hybrid vehicles

- `main.py` reads each drone's vehicle type from its MAVLink heartbeat and its
  cruise speed, waypoint radius and bank limit from the drone's parameters.
  Fixed-wing and quadplane bots fly like planes (they loiter in circles at a
  goal); copters hold position.
- **Bot id = the drone's MAVLink system id.** The GUI shows 01, 02, ... and
  sends those ids; `main.py` sorts the connected drones by system id and
  translates. Give every drone a **unique `SYSID_THISMAV`**. Each simulated bot
  also carries it as `Bot.sysid`.
- Ports no longer decide ids: a drone that connects late or not at all does not
  shift the others.

## Run it

Needs Python 3.8+ for `main.py`, [`uv`](https://docs.astral.sh/uv/) for the
server and Node.js for Live.

1. **Drones.** Real vehicles or SITL. Stream A to the server machine on UDP
   `14550`, stream B to the `main.py` machine on UDP `1455X`. Close any other
   ground station holding port `14550`.
2. **Swarm master.**
   ```bat
   run_main.bat <bind_ip> <num_bots>
   ```
   Or set `SWARM_IP` and `SWARM_NUM_BOTS` and run `python main.py`. It prints
   `Ip address: <ip>`; that is the server's `command_host`.
   `pip install -r requirements.txt` first.
3. **Server.** Set `command_host` in `gui/skybrush-server/etc/conf/swarm.jsonc`,
   then run `gui\skybrush-server\run_server.bat`.
4. **Live.**
   ```bash
   cd gui/live
   npm install          # first time only
   npm run start:electron    # desktop app, dev server on https://localhost:8080
   # or: npm start      # browser build on http://localhost:8080
   ```
   The dev server port is fixed at 8080 and the app fails to start if it is
   busy, rather than moving to another port. The two commands share that port,
   so run only one. Connect to the server at `localhost:5000`.

Optional SITL example (WSL, 5 quadplanes and 5 quadcopters):

```bash
ap-sitl-swarm --vehicle ./bin/arduplane:quadplane:5 --vehicle ./bin/arducopter:quad:5 \
  --num-drones-per-row 5 --home <lat>,<lon> --ip <windows-side IP>
```

## What you can do in Live

- **UAV toolbar:** takeoff (asks for altitude), loiter (QLoiter on a quadplane),
  **guided mode** (every vehicle type, sent as the `mode` command), return to
  home, land, arm / disarm, delete, global **stop**, remove / add a bot to the
  swarm (self-heal).
- **Swarm ops** (hub button in the map's top toolbar): goal with several
  waypoints, search, split, specific split, altitude staggering.
- **UAV grid:** each tile shows status, flight mode, airspeed and battery.
  There is a "Select all" checkbox above the grid.
- **Offline maps:** see below.
- Removed from the upstream Live: the 3D view, and the power on / sleep /
  reboot / power off buttons (and the matching `UAV-HALT`, `UAV-SLEEP`,
  `UAV-WAKEUP` server commands).

## Offline maps (`map_cache` extension)

Switch on "Use cached map tiles" in Toolbox -> Offline maps. Live then loads
tiles from `<server>/map-cache/_?url=...`; the server downloads each tile once,
stores it on disk and serves it again without internet.

- Enabled in `swarm.jsonc` with `"map_cache": { "enabled": true }`.
- Cache folder: `%LOCALAPPDATA%\CollMot Robotics\Skybrush Server\Cache\ext\map_cache`
  (capped at 2 GB by default, oldest tiles removed first).
- Put your Mapbox token in the `MAPBOX_ACCESS_TOKEN` environment variable, not in
  a file. Only known map-tile hosts are fetched; it is not an open proxy.
- `GET /map-cache/stats` shows the tile count, `POST /map-cache/clear` empties
  the cache.

## Configuration

- **`swarm.jsonc`:** `command_host` / `command_port`, the optional `id_map`
  (Skybrush id -> `main.py` id, only if they ever differ), `map_cache` options.
- **Config panel:** `http://<main.py ip>:8765/` edits swarm variables live
  (speeds, lookahead, `tick_interval_s`, `gui`, ...). Values persist across
  restarts. `gui` (matplotlib plot) is off for a headless run.
- **Search size:** the coverage area must be at least
  `(drones - 1) * 50 + drones * grid_space` metres, e.g. 550 m for 10 drones at
  10 m spacing. Smaller areas are rejected with a clear message.

## Troubleshooting

- **No drones in Live:** another program holds UDP `14550`, or stream A is not
  reaching this machine. Check the server log for `Connection at :14550`.
- **Swarm commands do nothing:** look for `swarm_bridge: sent ...` in the server
  log, then check `main.py` is running, `command_host` is the IP it printed,
  and UDP `12008` is not blocked.
- **`Number of Drones Connected: X/Y`:** the drones that failed to connect.
- **Electron window shows the wrong page:** something else is on port `8080`.
- **Bots lag behind the drones with `gui` off:** use the current `main.py`; its
  main loop used to spin and starve the guidance threads.

## Repository layout

| Path | What |
|---|---|
| `main.py`, `swarm.py`, `vehicle.py`, `variable.py`, `utils.py` | Swarm master |
| `swarm_tasks/` | Simulation, controllers and mission planners (library, see below) |
| `swarm_tasks/config_server.py` | Config panel and `/api/telemetry` |
| `gui/skybrush-server/` | GCS server, `ext/swarm_bridge/`, `ext/map_cache/` |
| `gui/live/` | Operator GUI, `src/features/swarm-ops/` |
| `send_command_example.py` | Reference client for the UDP commands |
| `CLAUDE.md`, `.claude/`, `.mcp.json` | Optional Claude Code / ruflo setup |

---

# swarm_tasks library (upstream)

The project aims to automate a decentralized swarm of autonomous mobile robots to carry out different tasks, with a focus on motion planning.

The system is completelly decentralized. Each robot only knows the approximate relative position of the robots within a speciific neighbourhood radius. The bots do not rely on inter-robot comminication, localization or memory, and possess only minimal local sensing & processing capabilities. They perform tasks collectively through 'swarm intelligence' algorithms.

Complex swarm applications are performed as a weighted combination of the following basc bheaviours:

- Aggregation
- Dispersion
- Exploration
- Line formation
- Shape Formation
- Surrounding an item
- Consensus

**[ Version 1.0.0 ]**

## Structure of the Repository
The repository is astructured as Python3 package. The subfolder ```swarm_tasks``` contains the following modules:

1. **Utilities (```utils```):** This module contains the class definitions and helper functions used in the rest of the repository. The most important of these is the ```Robot``` class. Each robot in th swarm is an instance of this class.

1. **Simulator (```simulation```):** The file ```simulation.py``` defines a simulation class that spawns the environment, obstacles, items and the robos, and simulates their actions and interactions. The ```visualizer.py``` contains a class to visualize this simulation (and other robot properties).
1. **Environments (```envs```):** Contains yaml files defiing the the ```worlds``` (Boundaries, obstacles, etc.) and the ```items``` (objects in the environment that robots interract with).

1. **Controllers (```controllers```):** The ```command.py``` file defines the ```Cmd``` class, which contains the simplest form of a command that a robot can execute, i.e. a velocity vector. Vector operations (adding commands, multiplying, etc.) on multiple commands produce the resutant commanded direction and speed of the robot. This leads to easy stacking up of simultaneous 'behaviors' for each robot.
Each controller implemented in this module returns an instance of ```Cmd```. The potential field motion planner has been implemented, which can take weights for different components of the environment as paramters. Using the potential field method, a base controller has been implemented, which executes basic inter-robot and obstacle avoidance behaviour. 

1. **Modules (```modules```):** All basic behaviours are implemented as functions that take the robot object as a parameter, and use the neighbour-positions to return a ```Cmd``` instance. The following behaviours have been implemented
    1. Aggregation: The robots cluster together 
    1. Dispersion: The robots spread out
    1. Exploration: The robots move around randomly
    1. Line formation: The robots form a straight line
    1. Chain formation: The robots form a chain
    1. Circle formation
    1. Surrounding: The robots surround an item
    
1. **Tasks (```tasks```):** Multiple behaviours can be combined by simply adding the outputs of their respective function. Such combinations can be used to implement complex realworld tasks. The following tasks have been implemented through weighted combinations of basic behaviours and non-deterministic finite-state machines:
    1. **Area Search:** The robots disperse and collectively explore an area.
    1. **Foraging/ Resource Gathering:** The robots search for & collectively transport resources to a 'NEST' at an unknown location. Alternatively, the robots gather all resources together at an undecided location.
    1. **Contamination Removal:** The robots create a perimeter around gradually growing contaminations and neutralize them
 
## Installation

Clone the package using the following command:

	git clone https://github.com/rmvanarse/swarm_tasks
Install the package from the root directory as follows:

	cd swarm_tasks
	python3 setup.py install

## Examples
The ```Examples``` folder contains Demo scripts for basic behaviours as well as full tasks. 

**Basic Behaviours:** Run the following from the root directory of the repository:

	python3 swarm_tasks/Examples/basic_tasks/<name_of_behaviour>.py
	
Examples for centroid-based aggregation, field-based dispersion, circle formation, line formation and perimeter behaviours have been provided.

**Full tasks:** Run the following from the root directory of the repository:

	python3 swarm_tasks/Examples/full_tasks/<name_of_task>.py

Examples for area search, foraging and contamination removal have been provided.

**Note:** Logs and parameters used for the tests are included in the ```logs``` folder


## Usage
**Note:** Regardless of explicit installation, the following currently requires to be in the root directory of the repository.

Import the package as follows:

	import swarm_tasks

Submodules and functions can be imported similar to standard python packages. For example:

	import swarm_tasks.simulation
	from swarm_tasks.utils import robot

The following are steps to initialize a basic swarm simulation (eg: Circle formation with obstacle avoidance):

```python
import swarm_tasks

from swarm_tasks.simulation import simulation as sim
from swarm_tasks.simulation import visualizer as viz
	
#Import required modules
import swarm_tasks.controllers.base_control as base_control
from swarm_tasks.modules.formations import circle


#Initialize Simulation and GUI 
s = sim.Simulation(env_name='rectangles')
gui = viz.Gui(s)
gui.show_env()

while(1):
	for b in s.swarm:
		#Basic behaviours
		cmd = base_control.base_control(b)
		cmd+= base_control.obstacle_avoidance(b)
		
		#Additional behaviours
		cmd+=circle(b,5)
		
		#Execute
		cmd.exec(b)
		
	gui.update()
```	
The ```test.py``` and ```test_tasks.py``` files can be used for easily testing and experimenting with the behaviours and tasks easily.

## Authors:
**Rishikesh Vanarse** ( [GitHub](https://github.com/rmvanarse), [Website](https://rmvanarse.github.io) ) 


