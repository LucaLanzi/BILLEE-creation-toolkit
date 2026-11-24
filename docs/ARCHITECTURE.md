# Architecture and Schema

## Layers

```
Blender UI and operators      blender/panels.py, operators.py, overlay.py
Scene data and sync           blender/props.py, authoring.py, sync.py
Scene to model                blender/extract.py    (meters are computed here, once)
Pure-Python model             core/model.py, assembly.py, validate.py, readiness.py
Math                          core/transforms.py, mass.py, units.py
Exporters                     exporters/urdf.py, xacro.py, srdf.py, stl.py, package.py
```

`core/` and `exporters/` never import `bpy`, so they run under any Python 3.11+.

## Data ownership

- **Project** holds modules and mounts. It is saved on the Blender scene as `scene.rab`, with `schema_version`.
- **Module** is a rover, arm, or accessory instance with its own prefix, links, joints, frames, and poses.
- **Link** is a rigid body made of **Components**.
- **Component** owns at most one mass specification, plus visual and collision geometry.
- **Collision proxies** are geometry only and can never add mass.
- **Joint** stores its canonical zero pose `T_P_J`. Preview values are stored separately and never exported.
- **Attachment frame** has an owner link, a pose, and a stable ID.
- **Mount** is a fixed connection between a parent frame and a child frame, with an offset.

## Identity and scene events

| Event | Behavior |
|---|---|
| Rename | Safe. References use stable IDs and object pointers, not names |
| Duplicate object | The copy is not treated as the owner, because ownership is checked by object pointer |
| Duplicate module | New IDs, new objects, and a unique prefix. The source is unchanged |
| Delete object | Reported as a blocking issue on the affected component or link |
| Undo, redo, save, reload | Scene data is restored by Blender. Displayed check results are recomputed |

## Transform conventions

`T_A_B` maps coordinates from B into A, so `p_A = T_A_B * p_B`.

```
T_P_J = inverse(T_W_P) * T_W_J
T_L_V = inverse(T_W_L) * T_W_V
T_W_armRoot = T_W_roverMount * T_offset * inverse(T_armRoot_armMount)
R = Rz(yaw) * Ry(pitch) * Rx(roll)
```

The project convention is right-handed, X forward, Y left, Z up. No axis rotation is applied on export. Geometry is
read through its parent chain relative to the owning link, so world placement and preview cannot affect exports.

## Units

Lengths are stored in Blender units so Blender's unit-aware fields display correctly.
`meters = Blender units * scene.unit_settings.scale_length`, applied once in `extract.py`.
Mass is kg, density is kg/m^3, inertia is kg*m^2, and angles are radians.

## Mass properties

Each link's inertia is computed about its center of mass, in axes parallel to the link frame:

```
M = sum(m_i)
c = sum(m_i * c_i) / M
I_c = sum(R_i * I_i * R_i^T + m_i * ((d_i . d_i) * Identity - d_i * d_i^T)),  d_i = c_i - c
```

Off-diagonal terms follow URDF, for example `ixy = -integral(x * y dm)`.

## BILLEE drop-in export

`exporters/billee.py` generates geometry and reuses the software stack's wiring. It copies the current
`robot_description` package, generates `robot_core.urdf.xacro`, and updates only values derived from the model. Wheels
are assigned by position: left runs rear to front (`l1` to `l3`), right runs front to rear (`r1` to `r3`). It only reads
the software repo, and it refuses to overwrite a `robot_description` folder it did not generate.

## Export safety

Files are generated in memory, written to a staging folder, then moved into place. A hash manifest detects edited
files. Export stops instead of overwriting them, and `config/user/` is never modified.
