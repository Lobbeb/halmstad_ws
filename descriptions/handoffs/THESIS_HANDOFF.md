## Thesis Handoff

This note captures the main conclusions from the recent dataset-labeling and OMNeT bridge discussion, so the work can be resumed on another machine without depending on chat history.

### Notation Decisions

- Use superscript `G` only for Gazebo variables.
- Use plain variables for OMNeT-mapped values.
- Introduce symbols in prose before presenting the equations.
- Avoid `\mathrm{}` for ordinary text labels if a simpler style is preferred.

### Dataset Capture: What Is Used

Relevant implementation:

- `/home/ruben/halmstad_ws/src/lrs_halmstad/lrs_halmstad/dataset/sim_dataset_capture.py`
- `/home/ruben/halmstad_ws/src/lrs_halmstad/lrs_halmstad/dataset/make_obb.py`

Topics used by `sim_dataset_capture.py`:

- Camera image: `/<uav>/camera0/image_raw`
- Camera intrinsics: `/<uav>/camera0/camera_info`
- Camera pose: `/<uav>/camera0/actual/center_pose`
- UGV pose: `target_pose_topic`, default `/a201_0000/amcl_pose_odom`

Fields used from the camera info topic:

- `f_x = K[0]`
- `f_y = K[4]`
- `c_x = K[2]`
- `c_y = K[5]`
- image width and height

Fields used from the UGV pose topic:

- position `(t_x, t_y, t_z)`
- orientation quaternion `(q_x, q_y, q_z, q_w)`

Fields used from the camera pose topic:

- camera position `C_w = (C_x, C_y, C_z)`
- camera quaternion `q_c = (q_x, q_y, q_z, q_w)`

Important implementation detail:

- The label cuboid vertical placement uses `target_base_z_m = z_b`, not the incoming UGV `t_z`, when building projected geometry.

### Dataset Label Geometry

The projected point set is formed by:

1. Sampling local cuboid points `(p_x, p_y, p_z)`.
2. Rotating them by UGV yaw and placing them in world coordinates:
   - `X_w = t_x + cos(psi) p_x - sin(psi) p_y`
   - `Y_w = t_y + sin(psi) p_x + cos(psi) p_y`
   - `Z_w = z_b + p_z`
3. Transforming the world point into the camera/body frame with the camera quaternion.
4. Applying the fixed body-to-camera remap `diag(1, -1, -1)`.
5. Projecting into image pixels with the pinhole model:
   - `u = c_x - f_x (b_y / b_x)`
   - `v = c_y - f_y (b_z / b_x)`

The stored projected point set is:

- `Q = {(u_j, v_j)}`

### AABB vs OBB

AABB:

- Derived from the enclosing axis-aligned rectangle of `Q`.
- Stored in YOLO detection format:
  - `(x_c, y_c, w, h)`
- Normalized by image width `W` and height `H`.

OBB:

- Not written directly by `sim_dataset_capture.py`.
- `sim_dataset_capture.py` stores `projected_points` in metadata.
- `make_obb.py` fits the minimum-area rotated rectangle to `Q` using `cv2.minAreaRect(...)`.
- The four corners are extracted with `cv2.boxPoints(...)`.
- The corner coordinates are then normalized by `W` and `H`.

Good mathematical phrasing:

- Let `Q = {(u_j, v_j)}` be the set of projected image points.
- Let `R(m, w, h, theta)` be an enclosing rectangle with center `m = (m_x, m_y)`, width `w`, height `h`, and in-plane rotation `theta`.
- Let `A(R) = w h`.
- For AABB, constrain `theta = 0`.
- For OBB, allow `theta` to vary freely.

OBB definition:

- `R^* = arg min_{R : Q subseteq R} A(R)`
- The OBB corners are the four vertices of `R^*`.
- A clean parameterization is:
  - `R^* = R(m^*, w^*, h^*, theta^*)`
  - `p_1^* = (-w^*/2, -h^*/2)`
  - `p_2^* = ( w^*/2, -h^*/2)`
  - `p_3^* = ( w^*/2,  h^*/2)`
  - `p_4^* = (-w^*/2,  h^*/2)`
  - `r_i^* = m^* + Rot(theta^*) p_i^*`

2D rotation matrix:

- `Rot(theta) = [[cos(theta), -sin(theta)], [sin(theta), cos(theta)]]`

### Sources Useful For Citations

For thesis citation support, the following were identified as the most relevant external references:

- OpenCV `calib3d` docs for the pinhole projection model.
- ROS `sensor_msgs/CameraInfo` docs for the intrinsic matrix layout.
- OpenCV `minAreaRect` / `boxPoints` docs for OBB construction.
- REP-103 for ROS frame conventions.

Important nuance:

- The exact sign-flipped projection equations used here come from combining the standard pinhole model with this repository's implementation-specific body-to-camera remap. The cleanest source for that exact final form is still the project code itself.

### OMNeT Bridge Summary

Relevant implementation:

- `/home/ruben/halmstad_ws/src/lrs_halmstad/lrs_halmstad/sim/gazebo_pose_tcp_bridge.py`
- `/home/ruben/halmstad_ws/src/lrs_halmstad/lrs_halmstad/sim/omnet_metrics_bridge.py`
- `/home/ruben/halmstad_ws/src/lrs_halmstad/launch/run_follow.launch.py`
- `/home/ruben/omnet_workspace/UAV_UGV/src/gazebo/GazeboPositionScheduler.cc`
- `/home/ruben/omnet_workspace/UAV_UGV/src/gazebo/GazeboDrivenMobility.cc`
- `/home/ruben/omnet_workspace/UAV_UGV/src/gazebo/OmnetMetricsServer.cc`
- `/home/ruben/omnet_workspace/UAV_UGV/omnetpp.ini`

Current flow:

1. ROS2 odometry is served over TCP by `gazebo_pose_tcp_bridge` on port `5555`.
2. OMNeT `GazeboPositionScheduler` polls with `GET`.
3. Pose snapshots are parsed and emitted as `gazeboPoseUpdated`.
4. `GazeboDrivenMobility` updates UAV/UGV positions from those snapshots.
5. `OmnetMetricsServer` computes distance, RSSI, SNIR, PER, and radio-only distance.
6. Metrics are streamed over TCP on port `5556`.
7. ROS2 `omnet_metrics_bridge` republishes them as `/omnet/*` topics.

Current launch-time model mapping in `run_follow.launch.py`:

- UGV model name: `robot`
- UAV model name: `dji0`

Current coordinate mapping in `omnetpp.ini`:

- `scaleX = 1`
- `scaleY = -1`
- `scaleZ = 1`
- `offsetX = 50`
- `offsetY = 50`
- `offsetZ = 0`

Thus, in the current bridged configuration:

- `x_k = x_k^G + 50`
- `y_k = -y_k^G + 50`
- `z_k = z_k^G`

Yaw mapping used by `GazeboDrivenMobility`:

- `psi_k^map = atan2(s_y sin psi_k^G, s_x cos psi_k^G)`
- `psi_k = wrap_{[-pi,pi]}(psi_k^map + psi_off)`

Velocity update:

- `v_k = (p_k - p_{k-1}) / (t_k - t_{k-1})`, when position changes and time advances
- otherwise `v_k = 0`

Live metrics computed by `OmnetMetricsServer`:

- geometric distance `d_k`
- free-space RSSI estimate
- SNIR from `RSSI - noise floor`
- PER from a sliding window of radio delivery/drop events
- radio-only distance by inverting the FSPL model

### Thesis Equation Style For The OMNeT Section

The final preferred style was:

- define Gazebo pose first:
  - `p_k^G = [x_k^G, y_k^G, z_k^G]^T`
  - `psi_k^G`
- define the scaling matrix `S` and offset vector `o`
- write:
  - `p_k = S p_k^G + o`
- keep OMNeT variables without superscripts

### If You Continue This Later

If resuming on another machine, the safest next steps are:

- keep this file as the lightweight handoff
- use repository file paths above to recover the exact implementation
- for thesis text, reuse the notation decisions here before adding more equations
