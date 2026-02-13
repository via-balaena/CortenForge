# Obstacle-Avoiding Robot — Phase 3 Product Spec

Wheeled robot with ultrasonic/IR range sensor array. Fuses range sensors into
a local occupancy grid (cf-spatial), plans collision-free paths (route-pathfind),
and replans in real-time when new obstacles are detected.

## Pipeline

```
Range sensors ─► Occupancy grid (cf-spatial) ─► A* path planning (route-pathfind)
    ─► Motor commands ─► Replan on new obstacles
```

1. **Sense** — Read range data from ultrasonic/IR sensor array (real or simulated via sim-sensor).
2. **Map** — Insert range readings into a local 2D occupancy grid (cf-spatial).
3. **Plan** — A* path planning from current position to goal through free space (route-pathfind).
4. **Act** — Convert path waypoints to differential-drive motor commands.
5. **Replan** — When new obstacles invalidate the current path, replan within the same control cycle.

## CortenForge Crates Used

| Crate | Purpose in this product |
|---|---|
| `sensor-types` | Range sensor data types (ultrasonic, IR distance readings) |
| `sensor-fusion` | Fuse overlapping range sensor readings for robust distance estimates |
| `sim-sensor` | Simulated range sensors with configurable noise, cone angle, max range |
| `sim-core` | Physics simulation stepping for wheeled robot |
| `sim-types` | Simulation data structures |
| `cf-spatial` | 2D occupancy grid for local obstacle mapping |
| `route-types` | Path and waypoint representations |
| `route-pathfind` | A* path planning on the occupancy grid |
| `mesh-types` | Robot and environment geometry representation |
| `mesh-io` | Load environment and robot meshes |

## Modes

- **Sim mode:** sim-sensor provides simulated range data, sim-core steps physics.
- **Hardware mode:** Real ultrasonic/IR sensors, same control code.

## Input

- Start position and goal position in 2D space.
- Environment with static and dynamic obstacles.

## Output

- Robot navigates from start to goal without collision.
- Path replans in real-time when new obstacles appear.

## Acceptance Criteria

1. Robot navigates an obstacle course without collision in both sim and hardware modes.
2. Path replanning completes within 100ms of detecting a new obstacle.
3. Occupancy grid correctly marks obstacles detected by at least two overlapping sensors.
4. Same control code binary runs in both sim and hardware mode.
5. Robot reaches the goal position within 10% of the optimal path length.

## Status

**Spec** — not yet implemented.
