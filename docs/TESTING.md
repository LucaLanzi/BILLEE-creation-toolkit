# Validation and Testing Report

Run on 2026-11-23 on Ubuntu 26.04 x86-64 with Blender 5.3.0 (build `02f0899d78e3`, Python 3.13.13).
Blender checks use a temporary profile and do not touch the real Blender configuration.

```sh
make gui    # Every local check, including a real Blender window
make ros    # Real ROS 2 Humble and Gazebo Fortress checks
make docs   # Screenshots, plus the mouse-drag and cancel test
```

## Local checks

| Check | Result |
|---|---|
| Core and exporter unit tests | 93 passed |
| Headless tests inside Blender | 38 passed |
| Extension build, manifest validation, ZIP validation | Passed |
| Install, enable/disable/re-enable three times, uninstall | Passed, no leftover handlers or classes |
| One-step install, reinstall, uninstall from a path with spaces | Passed on Linux |
| Automatic Blender discovery by the Linux installer | Passed with the Snap install |
| All panels draw on the Rover, Arm, and Assembly tabs | Passed in a real window |
| Undo and redo of Add Link | Passed in a real window |
| Robot tab opened by a simulated click | Passed |
| Modal joint drag, and Esc restoring the pose | Passed with simulated input |

Covered behavior includes 100 mm export at several unit scales, rotated primitives, parent inverses, rejected
stretched shapes with explicit mesh fallback, reflections, reframing, off-origin pivots, non-axis-aligned joints,
preview isolation, offset and rotated mounts, whole-assembly moves, proxy mass, composite inertia, invalid inertia,
graph cycles, duplicate instances, rename, deletion, save and reload, deterministic export, and stale readiness.

## ROS 2 and Gazebo gates

These run in the `billee-software-2027` dev container image using its locked pixi environment. The software repo is
mounted read-only and the container has no network. The demo package is exported through the Blender scene.

| Gate | Result |
|---|---|
| Package builds with `colcon` in a path with spaces | Passed |
| 1. Real `xacro` 2.1.1 expands every model | Passed |
| 2. `check_urdf` parses every model | Passed |
| 2. Expanded Xacro numerically equals the plain URDF | Passed |
| 3. Published TF matches forward kinematics from the URDF | Passed |
| 4. MoveIt arm planning | Passed. See the MoveIt check below |
| 5. Gazebo Fortress spawn with all joints publishing state | Passed |
| 5. Rover settles at the expected height and stays upright | Passed: z = 0.190 m, 0.000 rad tilt |
| 5. Arm reaches a joint-space goal through `joint_trajectory_controller` | Passed |
| 5. Wheels follow a velocity command and drive the rover | Passed: 2.6 m in 8 s |

The Gazebo gate found a real design flaw in an earlier demo: two independent rockers let the chassis pitch to the joint
limit. The demo was changed instead of relaxing the check.

## BILLEE drop-in check

`make dropin` exports the BILLEE rover through the Blender operators, then runs the stock model and the generated model
through the same unmodified `rover.launch.py` from `billee-software-2027` commit `e18b1f7`. The repo is mounted read-only.

| Check | Stock | Generated |
|---|---|---|
| Package builds, real `xacro` and `check_urdf` pass in sim and real modes | Passed | Passed |
| `diff_drive_controller` and `joint_state_broadcaster` active | Passed | Passed |
| All six wheel joints publish state | Passed | Passed |
| TF: `odom`, `base_footprint`, `base_link`, `camera_link`, optical frame | Passed | Passed |
| ZED sensor, IMU, odometry, and control plugins in the SDF | Passed | Passed |
| Wheel odometry and Gazebo ground truth publish | Passed | Passed |
| Drives forward on a forward command | Passed | Passed |
| Turns left on a positive yaw command | Passed | Passed |
| Real-hardware `ros2_control` block, including CAN IDs | Reference | Identical |

A literal folder replace in a temporary clone changes only `robot_core.urdf.xacro`, `robot_gz.urdf.xacro`, and
`camera.urdf.xacro` for the template rover. Controller values change when the model's wheel geometry differs.

The render sensor is removed from temporary copies during simulation because the check has no display. Its definition is
verified in the SDF. `real.launch.py` reaches hardware loading with both models, then both stop because the ODESC plugin
could not be created on this desktop, so **the generated model has not driven the real motors**.

## MoveIt check

`make moveit` exports the rover and arm through the Blender operators, then runs them on the unmodified `rover.launch.py`.
MoveIt 2.5.9 comes from the team's RoboStack channel in a separate layer. All 917 installed packages are pinned to their
current versions, and none were replaced.

| Area | Result |
|---|---|
| Rover drivetrain wiring is identical with the arm attached | Passed |
| Assembled model passes the URDF parser | Passed |
| Arm holds under hard rover acceleration with no arm controller | Passed |
| Rover and arm controllers active together | Passed |
| Plans and executes joint goals in Gazebo across CAN and USB controller groups | Passed |
| End effector moves through its own MoveIt group | Passed |
| Forward kinematics, position IK, and planning to a tool position | Passed |
| Detects a collision with the chassis and refuses to plan into it | Passed |
| Returns home, and the rover still drives with MoveIt running | Passed |
| Standalone arm plans and executes with mock hardware | Passed |
| Real `pxx` driver in mock mode accepts the CAN controller and follows a trajectory | Passed |

## Not run

- Real arm motors, including any RoboClaw motion.
- The generated model on the real ODESC drivetrain.
- Any run on Windows or macOS, including their installers.
- Linux Flatpak discovery.
- Drag-and-drop and Install from Disk.
- Opening the RViz window.
- Testing on the physical rover. The demo values are illustrative.
