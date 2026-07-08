# MARC 2026 - Manipulation Kit

The models and references you need to solve pick-and-place with **joint-angle control** of the 6-DoF robot arm.
The platform only accepts joint angles (IK/motion is the participant's responsibility), so this provides a starting point.

## Contents
| File | Description |
|---|---|
| `urdf/gen3_6dof_vision_2f140.urdf` | Robot kinematics model (6-axis arm + Robotiq 2F-140 gripper). **Self-contained** (no external meshes needed) -- loaded by `arm_kin` for FK/Jacobian |
| `arm_kin.py` | Kinematics engine (**placo / Pinocchio**). Loads the URDF and exposes FK + arm Jacobian + a damped-least-squares (DLS) closed-loop IK servo. EE tip = URDF `ee_frame_offset` (finger-pad center) |
| `arm_pick.py` | Baseline pick-and-place. A waypoint table (`pre -> descend -> grasp -> lift -> over -> place -> release -> back`) is servoed to with `arm_kin` (cart = straight-line DLS, joint = base-rotation arc), reading each tick's `arm/joint_states`. Grasp point xy comes from `grasp_xy` (the real object's arm_base position); depth/carry/basket/speed come from `pnp_params.json` (built-in defaults if absent) |
| `pnp_params.json` | Tuned pick parameters (grasp depth, carry/basket heights, gripper close, wrist roll `grasp_yaw`, speeds) |

## Dependencies
`arm_kin.py` uses **placo** (`pip install placo`), a Pinocchio-based kinematics engine. It is
pip-installable on the participant image's Python 3.10 (installed at build time -- the judging runtime
has no internet, so bake it into the image). The URDF is self-contained (no external meshes).

## Usage
```python
import arm_kin, arm_pick
# arm_kin.PlacoArmKinematics -> FK(4x4) + arm Jacobian; DLS servo toward an EE waypoint.
# arm_pick.run_pick_sequence(client, log, grasp_xy=(fwd, lat))  # closed-loop pick&place
#   -> publishes sensor_msgs/JointState on arm/joint_command, returns a grip-hold verdict.
```
- Control interface: `sensor_msgs/JointState` (joints `joint_1..6` + gripper). Pace the loop by the incoming
  `arm/joint_states` feedback (closed loop), not a fixed wall-clock rate. Detailed ICD -> developer guide/SDK.
- The robot USD is **included in the platform** and is not provided separately (participants control it via ROS 2 joint commands and compute kinematics from the URDF).

## Two ways to use it
- **Using them to the maximum**: leave the picking sequence (`arm_pick.py`) untouched and use it
  as-is like a finished tool -- compute only the object's grasp point (`grasp_xy`) and pass it in;
  the baseline picking motion runs and can earn partial score.
- **Using them minimally**: take only `arm_kin` (FK/Jacobian/DLS) and the URDF as materials and
  design your own arm-control approach.

## License / attribution
- `gen3_6dof_vision_2f140.urdf` -- derived from Kinova Gen3 + Robotiq 2F-140. **Verify the original license + provide NOTICES attribution** (Kinova ROS URDF is typically BSD-3).
- `arm_kin.py` / `arm_pick.py` -- MARC 2026 (IoTCOSS). `arm_kin.py` links **placo** (Pinocchio; see its own license).

> The manipulation trainer (a practice environment) ships in the same platform image -- run it with
> `bash simulation-platform/marc.sh manip-trainer`. See the developer guide (Technical Guide ->
> Manipulation) for details.
