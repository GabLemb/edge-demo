# Swarm Arena — Distributed Edge Resilience Demo

A ROS2 simulation that demonstrates absolute swarm resilience and decentralized
state replication across multiple autonomous robots.

Three Observer Robots patrol a 6×6m arena tracking moving targets. Each robot
maintains a local copy of the entire swarm's observation history through a mock
distributed DataLayer with replication factor 3. The system is designed to
survive arbitrary node failures: kill any one robot mid-mission and the
surviving nodes continue patrolling with zero data loss on the historical
tracking record.

## Architecture
┌─────────────────────────────────────────────────────────┐
│                  Gazebo Arena (6×6m)                    │
│   ┌─────────┐    ┌─────────┐    ┌─────────┐             │
│   │Observer1│    │Observer2│    │Observer3│             │
│   └────┬────┘    └────┬────┘    └────┬────┘             │
└────────┼──────────────┼───────────────┼─────────────────┘
│              │               │
ROS2 topics: /observer_N/{cmd_vel, scan, target_observations}
│              │               │
┌─────▼──────┬───────▼───────┬───────▼──────┐
│ DataLayer  │   DataLayer   │   DataLayer  │
│ (mock #1)  │   (mock #2)   │   (mock #3)  │
│ port 5001  │   port 5002   │   port 5003  │
│ SQLite #1  │   SQLite #2   │   SQLite #3  │
└─────┬──────┴───────┬───────┴───────┬──────┘
│              │               │
└──── /swarm/replicate (broadcast) ─┘

The system separates two concerns:

**Reflex layer (ROS2):** real-time perception, locomotion, and obstacle
avoidance. Each Observer runs a reactive LiDAR-based patrol loop and a
field-of-view simulator that generates target observations.

**Memory layer (mock DataLayer):** durable, replicated state. Every observation
produced by any Observer is broadcast to every node and persisted locally.
Replication is eventually-consistent and deduplicated by
`(observed_by, timestamp, target_id)`. The HTTP API on each node exposes the
local view of the global state.

The DataLayer in this repository is a Python mock intended to be a drop-in
replacement target for a real distributed runtime. The contract is the
JSON observation format and the deduplication key — any backend implementing
those can substitute this mock.

## Components

| File | Role |
|---|---|
| `swarm_arena/random_patrol_node.py` | Reactive LiDAR avoidance, randomized wandering, asymmetric turn bias to prevent symmetric deadlocks |
| `swarm_arena/target_mover_node.py` | Drives 5 target entities around the arena with wall-bouncing physics |
| `swarm_arena/target_tracker_node.py` | Per-observer field-of-view simulator; emits JSON observations via ROS2 topic |
| `swarm_arena/data_layer_node.py` | Mock distributed DataLayer: subscribe → replicate → dedupe → persist (SQLite) → expose HTTP |
| `worlds/arena.world` | Gazebo SDF: 6×6m arena, 3 obstacles, 5 targets, gazebo_ros_state plugin |
| `launch/swarm_full.launch.py` | Orchestration with timed staggered startup |
| `scripts/chaos_test.sh` | Convenience script to kill a node and verify replication invariants |

## Requirements

- Ubuntu 20.04
- ROS2 Foxy
- Gazebo Classic 11
- TurtleBot3 packages
- Python 3.8

```bash
sudo apt install \
    ros-foxy-desktop \
    ros-foxy-turtlebot3 \
    ros-foxy-turtlebot3-gazebo \
    ros-foxy-gazebo-ros-pkgs \
    python3-flask \
    sqlite3
```

## Build

```bash
cd ~/ros2_ws/src
git clone <this-repo> swarm_arena
cd ~/ros2_ws
colcon build --symlink-install --packages-select swarm_arena
source install/setup.bash
```

## Run

```bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/foxy/share/turtlebot3_gazebo/models
ros2 launch swarm_arena swarm_full.launch.py
```

After ~25 seconds, you will see:
- Three TurtleBot3 robots spawned and patrolling autonomously
- Five colored target cylinders bouncing around the arena
- Three DataLayer nodes accumulating and replicating observations

## Verify replication

```bash
for p in 5001 5002 5003; do
  echo -n "Port $p: "
  curl -s http://localhost:$p/status
  echo
done
```

The three nodes should report similar `total_observations` counts (with minor
jitter). Any divergence is bounded by network propagation delay.

## Chaos test

```bash
# Cut power to observer_1 (simulates a hardware kill-switch)
pkill -9 -f "ns:=/observer_1"

# The remaining two DataLayers retain the full pre-kill history
curl -s http://localhost:5002/status
curl -s http://localhost:5003/status

# Wait 30 seconds and verify they continue tracking and replicating
sleep 30
curl -s http://localhost:5002/status
curl -s http://localhost:5003/status
```

Expected behavior:
- Port 5001 becomes unreachable immediately
- Ports 5002 and 5003 keep their pre-kill observation count intact
- After 30 seconds, both surviving nodes show new observations and remain
  pairwise consistent

## Inspecting the persisted state

Each DataLayer node persists its view to a SQLite database. After a run:

```bash
sqlite3 /mnt/nvme/myrmic_mock/observer_1.db \
  "SELECT target_id, COUNT(*) FROM observations GROUP BY target_id;"
```

To verify replication is exact, hash each node's keyset:

```bash
for db in observer_1.db observer_2.db observer_3.db; do
  echo -n "$db: "
  sqlite3 /mnt/nvme/myrmic_mock/$db \
    "SELECT observed_by||'|'||timestamp||'|'||target_id FROM observations ORDER BY 1;" \
    | md5sum
done
```

Identical hashes ⇒ exact replication.

## Tuning

All node parameters can be set at runtime:

```bash
ros2 param set /observer_1/random_patrol linear_speed 0.10
ros2 param set /observer_1/random_patrol safe_distance 0.6
ros2 param set /observer_1/target_tracker detection_radius 3.0
```

Defaults are chosen for a 6×6m arena with TurtleBot3 burger robots. For larger
arenas, increase `detection_radius` proportionally.

## License

Apache-2.0
EOF