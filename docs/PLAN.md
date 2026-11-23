# Project Plan and Status

## Decisions

1. **One extension for every platform.** Pure Python with no compiled wheels. Targets Linux x64, Windows x64 and arm64,
   and Apple Silicon. Intel Macs are out of scope.
2. **Match the team's stack.** Exports target ROS 2 Humble, Gazebo Fortress, and `ign_ros2_control`, as locked in
   `billee-software-2027`.
3. **One shared robot model.** Rover, Arm, Assembly, and every exporter use the same pure-Python representation.
4. **Own binary STL writer.** Deterministic output, with no context-sensitive operators or scene changes.
5. **Staleness by content hash.** A model change demotes any recorded check.
6. **Conservative collision defaults.** Generated SRDF disables adjacent links only.

## Status

| Slice | Status |
|---|---|
| Transforms, units, mass properties | Done and tested |
| Model, assembly, validation, readiness | Done and tested |
| URDF, Xacro, STL, ROS package | Done and tested with real ROS 2 Humble tools |
| Gazebo and `ros2_control` configuration | Done and tested in Gazebo Fortress |
| Blender interface, overlays, demo | Done and tested on Linux |
| Installers and README | Done. Linux tested |
| MoveIt SRDF and launch | Generated, not tested. MoveIt is not in the BILLEE environment |
| Windows and macOS | Not yet run |

## Next steps

1. Run the installers on a Windows PC and an Apple Silicon Mac.
2. Add MoveIt 2 for Humble to an environment and test arm planning.
3. Author the real BILLEE rover and arm with measured values.
