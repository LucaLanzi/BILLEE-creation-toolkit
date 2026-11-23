# Known Limitations

## Not yet verified

- **MoveIt** has not been run because it is not installed in the BILLEE Humble environment. Treat the generated SRDF,
  MoveIt configuration, and `moveit.launch.py` as a starting point.
- **Windows and macOS** have not been run, including their installers.
- **The RViz window** has not been opened. The TF data it displays has been checked.
- **Drag-and-drop install** and **Install from Disk** have not been exercised.

## Not included in this release

- No gripper builder or standalone arm test pedestal.
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
- Generated MoveIt configuration disables collision checks for directly connected links only.
- `robot_state_publisher` logs a KDL warning when the root link has inertia. TF is unaffected. KDL-based dynamics ignore
  that link's inertia.
- A valid export or center-of-mass marker does not show that a robot resists tipping or can perform a motion.
- Linked-library and instanced collection objects are not supported as robot components.
