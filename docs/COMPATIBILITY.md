# Compatibility

Verified on 2026-11-23.

## Blender

| Item | Value | How verified |
|---|---|---|
| Target | Blender 5.3.0 stable | Run locally: build `02f0899d78e3`, release cycle `release` |
| Bundled Python | 3.13.13 | Run locally |
| Extension schema | 1.0.0 | Accepted by `blender --command extension validate` |
| Published builds | Linux x64, Windows x64 and arm64, macOS arm64 | [Blender 5.3 release folder](https://download.blender.org/release/Blender5.3/) |

Intel Macs are out of scope.

## Robotics stack

The target matches the locked environment in
[`billee-software-2027`](https://github.com/BILLEE-BroncoSTAR/billee-software-2027) (`ros2_ws/pixi.toml` and `pixi.lock`).

| Component | Version | Status |
|---|---|---|
| ROS 2 | Humble, through pixi and RoboStack | Tested |
| Gazebo | Fortress (`libignition-gazebo6` 6.16.0) | Tested |
| xacro | 2.1.1 | Tested |
| ros2_control | `controller_manager` 2.54.0 | Tested |
| Controllers | `joint_trajectory_controller` 2.53.1, velocity controllers | Tested |
| Gazebo control | `ign_ros2_control` 0.7.19, `ign_ros2_control/IgnitionSystem` | Tested |
| Gazebo launch | `ros_gz_sim` 0.244.24 | Tested |
| MoveIt 2 | 2.5.9 from `robostack-humble`, in a separate layer | Tested in Gazebo |
| Arm CAN driver | `pxx/PxxSystemHardware`, position command | Tested in mock mode |
| Arm USB driver | `roboclaw_hardware_interface` at `ae00980`, velocity command | **Blocked: no velocity state** |

MoveIt 2.5.10 requires a newer ROS build than the team's lock. MoveIt 2.5.9 matches it and the team's `moveit2` fork.
The drop-in `robot_description` does not depend on MoveIt. Only `billee_arm_moveit_config` does.

The 5203 19.2:1 encoder count comes from the
[goBILDA specification](https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-6mm-d-shaft-312-rpm-36mm-gearbox-3-3-5v-encoder/)
and [motor listing](https://core-electronics.com.au/5203-series-yellow-jacket-planetary-gear-motor-1921-ratio-24mm-length-8mm-rex-shaft-312-rpm-33-5v-encoder.html).

The BILLEE drop-in export was verified against `billee-software-2027` branch `PreRelease-DashBoardDev`, commit `e18b1f7`.

## Related tools

[Phobos](https://github.com/dfki-ric/phobos) is a BSD-3-Clause Blender add-on that creates URDF, SDF, and SMURF models.
Its README states testing with Blender 3.3 LTS, so it was not used as a dependency for Blender 5.3. No Phobos code is
included in this project.

## Dependencies and license

The extension uses only Blender's bundled Python modules and the standard library. It is licensed GPL-3.0-or-later,
as required for code that links against Blender's Python API.
