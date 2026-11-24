# Testing

Last run on 2026-11-23 on Ubuntu x86-64 with Blender 5.3.0, against `billee-software-2027` `PreRelease-DashBoardDev`
at `afb9467`, using its locked dependencies. That repo was never modified.

```sh
make gui      # Unit tests and every Blender check, including a real window
make ros      # Generic package on ROS 2 Humble and Gazebo Fortress
make stack    # Build the private BILLEE stack with MoveIt 2.5.9. Run once
make billee   # Rover, arm, and MoveIt exports on that stack
```

## Results

| Suite | Result |
|---|---|
| Unit tests | 99 passed |
| Tests inside Blender | 38 passed |
| Local checks: build, install, reinstall, uninstall, panels, undo, joint drag | 17 of 17 passed |
| Generic package: xacro, URDF, TF, Gazebo spawn, arm, and wheels | Passed |
| BILLEE stack, using the dashboard's own launch arguments | 35 checks passed |

## BILLEE stack

**Rover.** The stock and generated `robot_description` run through the same `rover.launch.py`. Both activate the
controllers, publish six wheel states and the full TF chain, report odometry, drive forward, and turn left. This confirms
behavior, not matched dynamics. The template's masses and shapes are placeholders, so travel distances differ.

**Arm.** The generated `arm_description` passes the team's own `test_import_urdf.py`. MoveIt then runs through the team's
`arm.launch.py` with mock CAN and RoboClaw hardware:

- Refuses to move while axes are `verified: false`, and the arm stays still.
- Plans and executes joint goals across the CAN and RoboClaw axes, and moves the end effector.
- Solves forward kinematics and position IK, and plans to a tool position.
- Detects a collision with the chassis and refuses to move into it.
- Sends RoboClaw velocity during motion and zero after motion or cancel.
- Keeps one TF tree, and the rover still drives with MoveIt running.
- Works with the arm alone, without the rover.

## Not tested

- Real motors: ODESC drivetrain, `pxx` CAN axes, or RoboClaw axes.
- The MoveIt bridge on the Jetson.
- The solenoid. `arm_solenoid_controller` does not activate with the team's mock hardware.
- The ZED render sensor in simulation, because the check has no display.
- The Windows and macOS installers, and Blender's drag-to-install.
