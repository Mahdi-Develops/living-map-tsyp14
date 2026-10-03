# The Living Map — TSYP14 RAS×AESS Challenge

A two-robot "spatial memory" system for GPS-denied disaster environments:
a **Writer drone** explores and drops beacons at hazards/victims, an
**Outside Network Area** bridges the disconnected zone to the real world,
and an **Executor rover** is briefed and acts on inherited beacon knowledge.

Environment chosen: **Urban Search & Rescue** (collapsed building).

## Architecture

```
 [Writer drone]                [Outside Network Area]            [Command Post]
  explores (sweep)               1. RECEIVE  <--- entrance only      live map
  senses events  ----beacon----> 2. TRANSLATE (xy -> GPS)   ----->   (HTML, auto-
  drops beacons                  3. CARRY    ----------------------> refreshing)
  returns to entrance             4. BRIEF    ---mission--->  [Executor rover]
                                                                 navigates to
                                                                 beacons, acts,
                                                                 reports back
                                                                 (via Outside
                                                                  Network only)
```

* **No direct link** ever exists between either robot and the Command Post —
  `outside_network_node` is the sole relay in both directions (ROS topics
  `/writer/to_outside_network` and `/executor/to_outside_network` are its only
  inbound robot-side channels; `/command_post/*` are its only outbound
  Command-Post-side channels).
* The Writer hands off its beacon log only once, when back within the
  entrance zone — modelling the short-range RF link between robot and the
  Outside Network Area's receiver.

## Packages

| Package | Contents |
|---|---|
| `living_map_description` | URDF/xacro for the Writer drone (kinematic hover — gravity disabled, horizontal planar-move plugin, 2D lidar) and Executor rover (differential drive, 2D lidar) |
| `living_map_gazebo` | `usar_building.world` (collapsed-building layout, 1.2m walls so the drone cruising at 1.5m flies clear of them, 3 ground-truth event markers) + spawn launch |
| `living_map_system` | The four logic nodes: `writer_node`, `executor_node`, `outside_network_node`, `command_post_node`, plus the `Beacon` message schema |
| `living_map_bringup` | Top-level launch tying everything together |

## Beacon message

JSON-encoded stand-in for the compact RF payload (`living_map_system/beacon_message.py`):

```json
{"id": "B1", "what": "hazard", "x": -6.0, "y": -5.0, "distance": 3.9, "when": 1791063219.0, "version": 1}
```

`when` is what makes it age: `fresh` (<5 min), `aging` (<15 min), `stale`
(beyond). The Executor trusts fresh beacons outright and re-verifies stale
ones against its own sensing before acting — exactly the "ages honestly"
requirement.

## Frame translation

`outside_network_node.local_to_gps()` anchors the entrance at a fixed
real-world GPS reference point and converts the Writer's local (x, y) metres
into lat/lon via an equirectangular (flat-earth) approximation. This is the
real-world equivalent of a surveyed entrance point; swapping in a true
geodesic projection is a drop-in change if sub-metre accuracy is needed.

## Outside Network ↔ Command Post link

Per the challenge Q&A, the satellite/long-range hop is simulated as a ROS
topic (`/command_post/beacons`), explicitly documented here as the
real-world stand-in for a LoRaWAN/SATCOM uplink — swapping in `rosbridge` +
MQTT over an actual radio link is a transport-layer change only; the
RECEIVE/TRANSLATE/CARRY/BRIEF logic does not change.

## Running it

```bash
# ROS2 Humble + Gazebo Classic + this workspace must be sourced
cd living_map_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch living_map_bringup living_map.launch.py
```

The live map renders to `live_map.html` in the workspace root and
auto-refreshes every 2s.

### Logic-only validation (no Gazebo)

`test_chain.py` exercises the full RECEIVE → TRANSLATE → CARRY → BRIEF →
navigate → report chain using a synthetic Writer handoff and a lightweight
kinematic stand-in for the Executor's body, without depending on Gazebo
physics timing at all:

```bash
ros2 run living_map_system outside_network_node &
ros2 run living_map_system command_post_node &
ros2 run living_map_system executor_node &
python3 test_chain.py
```

## Failure cases

1. **Writer never reaches the entrance** (battery dies / robot destroyed
   mid-mission): beacons it already dropped remain physically in the world
   and keep broadcasting — but its *log* of what it saw never reaches the
   Outside Network, since handoff only happens at the entrance boundary.
   Mitigation (not yet implemented): periodic short-range broadcast attempts
   throughout exploration, not just at the end, so a partial log survives
   even if the Writer never returns.
2. **Stale beacon, no ground truth to verify against**: if the Executor
   arrives at a beacon older than the staleness threshold and its own
   sensors can't confirm the hazard/victim is still there, it skips the
   target rather than acting on possibly-outdated information — trading
   mission completeness for safety. Demonstrated directly in
   `test_chain.py`'s runs.
3. **GPS-translation anchor drift**: the entrance GPS anchor is a single
   fixed reference point; if it's surveyed incorrectly (or the building
   has moved/collapsed further since survey), every translated beacon
   inherits that offset. No current cross-check exists — a real deployment
   would want a redundant anchor (e.g. two known GPS points at the entrance)
   to detect anchor error.
4. **Environment clock instability** (discovered during this build): on a
   host with an unstable system clock (observed here: a Hyper-V/WSL2
   time-sync hiccup causing multi-thousand-second jumps), any
   wall-clock-based timing — beacon aging, exploration timeouts — can
   misbehave, since `time.time()` is not monotonic across such jumps.
   Fixed for pure control-loop timing by switching to `time.monotonic()`
   in `writer_node`/`executor_node`; beacon *aging* intentionally keeps
   `time.time()` since real-world beacon freshness must reflect wall-clock
   reality, not host uptime — so a genuinely unstable deployment clock
   remains a real operational risk worth monitoring in the field (e.g. via
   NTP sync-status checks before trusting beacon ages).
