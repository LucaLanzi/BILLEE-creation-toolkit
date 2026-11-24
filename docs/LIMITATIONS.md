# Limitations

## Before using real hardware

- **No generated package has driven real motors.** Rover and arm were tested in Gazebo and with mock arm hardware.
- **Every arm axis is still `verified: false`** in `arm_axes.yaml`, so MoveIt refuses to move it. Verify axes on the bench first.
- **The bridge has no hardware watchdog.** The RoboClaw controller holds its last velocity. The bridge sends zero on
  cancel, errors, and normal shutdown, but a hard kill cannot. Keep the E-stop within reach.
- **Run the bridge on the rover** for real hardware, with `arm_bridge.launch.py`, so its velocity loop doesn't depend on the radio link. This has not been tested on the Jetson.
- **The solenoid is not controlled by MoveIt.** The team's `arm_solenoid_controller` also fails to activate in mock mode.
- **The front wheel CAN IDs disagree in the software repo.** `ros2_control.urdf.xacro` uses FR=3 and FL=4, and
  `odesc/config/node_map.yaml` uses FR=4 and FL=3. The driver uses the URDF, and the export preserves it.

## Model rules

- The rover needs six driven wheels with one radius, three per side, spinning about Y.
- The arm needs one joint for each `joint_arm_*` in `arm_axes.yaml`, and limits inside that file's limits.
- Other rover and arm joints export as fixed, because the software has no controller for them.
- Closed loops, such as a rocker-bogie differential, block export because URDF only describes trees.
- The end effector is fixed in TF, matching the team's model, and movable only in MoveIt's model.
- Five-axis IK solves tool position only.
- Collision checks are disabled only for directly connected links. Real geometry may need more reviewed pairs.
- The team's arm camera mount is unchanged, so check it against your tool frame.

## Not included

- Coordinated base-and-arm planning, terrain-aware suspension, actuator sizing, or new hardware drivers.
- URDF import and live-linked assets.
- MoveIt inside `billee-software-2027`. It runs from this repo's private stack until the team adds it.

## Blender

- Blender 5.3 can't switch sidebar tabs by script, so click the Robot tab once.
- Mass is in kg and joint limits in radians. Number fields accept math, such as `250/1000` or `radians(45)`.
- Exported lengths are rounded to 7 significant digits, because Blender stores single-precision values.
- Linked-library and instanced objects are not supported as components.
- A valid export or center-of-mass marker doesn't prove the robot won't tip.
