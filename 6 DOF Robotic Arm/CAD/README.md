# CAD

3D-printed parts grouped by sub-assembly. All STL, metric.

| Folder | Parts |
|---|---|
| `Assembly/` | `Bare_assembly.stl`: full arm bare-assembly reference |
| `Base/` | `J1_Base`, `J1_backplate`, `J1_rotation_shaft`, `J1_turret_motor_holder` |
| `Shoulder/` | `J1_turret`, `J2_limit_switch_cover`, `J2_stopper_block` |
| `Upper_Arm/` | `J2J3_upper_arm`, `J2_shaft`, `J2_wire_cover`, `J3_limit_switch_cover`, `J3_pulley1`, `J3_pulley2` |
| `Elbow/` | `J4_bearing_backplate`, `J4_elbow`, `J4_elbow_cover`, `J4_limiter` |
| `Forearm/` | `J4_pulley`, `J5_forearm`, `J5_pulley` |
| `Wrist/` | `J5_wrist` |
| `Gripper/` | `G_mainbody`, `G_main_body_lid`, `G_Jaw`, `G_finger`, `G_gear`, `G_coupler`, `G_guide_cover` |

## Notes

- Limit-switch covers, wire covers, and the J4 limiter are non-structural cosmetic / cable-management parts.
- Gripper drives a BLDC actuator through `G_gear` + `G_coupler` to actuate `G_Jaw` / `G_finger`.
- Joint kinematics + DH parameters are documented in the top-level `README.md`.
