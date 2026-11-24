# Known Limitations

## Not yet verified

- **Real arm motors** have not been driven. MoveIt is tested in Gazebo, with mock hardware, and with `pxx` in mock mode.
- **The generic package's** older MoveIt files have not been run. Use the BILLEE arm export for MoveIt.
- **Windows and macOS** have not been run, including their installers.
- **The RViz window** has not been opened. The TF data it displays has been checked.
- **Drag-and-drop install** and **Install from Disk** have not been exercised.

## BILLEE drop-in export

- It requires exactly six driven wheels with one shared radius, three per side, spinning about Y.
- Non-wheel rover joints, such as suspension, export as fixed because the rover software has no controller for them.
- It has been tested in simulation with the unmodified launch files, not on the real ODESC drivetrain.
- CAN IDs are copied as they are. In commit `e18b1f7`, `ros2_control.urdf.xacro` and `odesc/config/node_map.yaml`
  disagree on the front wheels: the URDF uses FR=3 and FL=4, while the map uses FR=4 and FL=3. The driver uses the URDF.
- The bundled wiring can go out of date. A local checkout of `billee-software-2027` is preferred when one is found.

## BILLEE arm and MoveIt

- **RoboClaw axes cannot run under the trajectory controller yet.** The driver pinned by `billee-software-2027` exposes
  velocity commands and position state only. `joint_trajectory_controller` 2.53.1 refuses that combination in testing.
  It needs a velocity state, through a driver change or an adapter.
- The `roboclaw_hardware_interface` and `roboclaw_serial` submodules are not checked out or built in the local workspace.
- Axes 4, 5, and the end effector need two RoboClaw 2x7A units, because each has two motor channels.
- The driver takes whole encoder counts, so the 5203 19.2:1's 537.7 counts per output turn is stored as 538.
- CAN IDs, RoboClaw addresses, and which axes use P17 or P20 are not set in the template.
- A five-axis arm uses position-only IK. It cannot reach every tool position and orientation.
- Arm shapes, masses, limits, and acceleration values are placeholders.
- MoveIt is not in the software repo's environment. It runs from a separate layer until the team adds it.
- The local pixi environment is older than the repo's current `pixi.lock`, and the layer was tested against what is installed.

## Not included in this release

- No standalone arm test pedestal in Gazebo. The standalone arm uses mock hardware.
- No custom viewport gizmos. Frames use Blender's native transform tools plus overlay guides.
- No URDF import or live-linked asset libraries. Reuse modules with Duplicate Module.
- No coordinated base-and-arm planning, terrain-aware suspension prediction, actuator sizing, or hardware drivers.

## Closed mechanical loops

Standard URDF only describes trees. A closed loop, such as a rocker-bogie differential, is reported and blocks export.
Two independent side rockers with no differential export correctly but let the chassis pitch freely in simulation.
The demo avoids this with a rigid rear axle and a passive pivoting front axle.

## Behavior to know

- Blender 5.3 cannot switch the sidebar tab by script, so click the Robot tab once.
- Mass fields are kg and joint limit fields are radians or scene length. Blender's math input covers other units.
- Exported lengths are rounded to 7 significant digits because Blender stores single-precision values.
- The mass-overlap warning uses approximate bounding boxes within a link, so it can warn about shapes that do not touch.
- The collision overlay is a display aid and performs no collision test.
- Xacro parameters cover prefixes, mount poses, primitive dimensions, mass or density, and joint limits. Joint and
  component placements are fixed values and do not follow dimension changes.
- Generated MoveIt configuration disables collision checks for directly connected links only. Real geometry may need
  more reviewed pairs.
- `robot_state_publisher` logs a KDL warning when the root link has inertia. TF is unaffected. KDL-based dynamics ignore
  that link's inertia.
- A valid export or center-of-mass marker does not show that a robot resists tipping or can perform a motion.
- Linked-library and instanced collection objects are not supported as robot components.
