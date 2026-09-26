# Support-Chain Final Handoff

This note closes the practical ROS 2 / Gazebo support-chain phase for thesis writing and handoff. It documents what is implemented, how to run it, which topics prove it works, what evidence still needs to be collected, and what should remain future work.

## Implemented System

The support-chain topology is:

```text
dji1/dji2 support observations -> dji0/UAV1 aggregation -> UGV-facing support information
```

The direction is intentionally asymmetric/simplex for this thesis phase. The support UAVs send observations upward through `dji0`; the UGV does not send commands back through this support chain.

Main roles:

- `dji0` is the main UAV associated with the UGV mission.
- `dji1` and `dji2` are support UAVs.
- `dji1/dji2` follow `dji0` using fixed odom-based support slots.
- The current support geometry is forward-left and forward-right around `dji0`.
- Same-altitude support is the default.
- This is the current fixed odom-slot baseline.

Support observation chain:

- `dji1/dji2` have camera topics.
- `dji1/dji2` support detector topics publish.
- Support observations flow upward from `dji1/dji2` to `dji0`, then onward to the UGV-facing topics.
- Detector training/tuning is not part of this final support-chain phase.

## dji0 Aggregation

`dji0` publishes the structured aggregation/selection summary on:

```text
/coord/dji0/support_observation_summary
```

This is the `dji0`-side summary of support observations. It includes selected source, mux strategy, source states/freshness, relation metadata, and detection state. It also publishes useful state when the result is `NO_DET` or `NO_INPUT`.

## UGV-Facing Information

The UGV receives structured support information on:

```text
/coord/ugv/support_observation_summary
```

The forwarded summary includes metadata such as:

```text
forward_owner=dji0
forward_stage=dji0_to_ugv
forwarded_from_topic=/coord/dji0/support_observation_summary
```

The UGV-side human-readable awareness/status hook publishes on:

```text
/coord/ugv/support_awareness_status
```

This is status/awareness only. It does not control Nav2, does not publish `cmd_vel`, and does not make the UGV actively replan. It exists so the UGV side can expose that support information is being received and summarize the current support-chain state.

The UGV-side advisory hook publishes on:

```text
/coord/ugv/support_path_advisory
```

This is also status/advisory only. It is intended as a clean future integration point for Nav2, costmaps, or replanning logic. Current messages use `suggested_action=monitor_only` and `replanning_enabled=false`; no active UGV behavior is triggered by this topic in the thesis implementation.

## Optional Camera Scanning

An optional support camera scanning/capture mode exists for `dji1/dji2`. It is disabled unless explicitly enabled through launch arguments.

Purpose:

- Let `dji1/dji2` act more like active support eyes.
- Support later image collection, data analysis, training material, or object-detection validation.
- Pan slowly over a limited front/side range.
- Avoid 360-degree spinning.
- Avoid moving UAV bodies for scanning.
- Preserve support-follow motion.

Scanner outputs when enabled:

```text
/coord/support/camera_scan_status
/dji1/update_pan
/dji2/update_pan
/dji1/update_tilt
/dji2/update_tilt
```

Use `support_bridge_gimbal:=true` when enabling scanning so the support gimbal joint command topics are bridged intentionally.

## Support Detector Model Switching

The support detector instances are instance-safe:

```text
/coord/support/dji1/*
/coord/support/dji2/*
```

Both support UAVs use the same detector settings by default. These can be changed from the command line without editing code.

Shared support detector overrides:

```bash
./run.sh tmux_support_chain warehouse mode:=follow gui:=true \
  support_detector_backend:=onnx_cpu \
  support_detector_onnx_model:=/absolute/path/to/support_model.onnx
```

Shared Ultralytics weights override:

```bash
./run.sh tmux_support_chain warehouse mode:=follow gui:=true \
  support_detector_backend:=ultralytics \
  support_yolo_weights:=/absolute/path/to/support_model.pt
```

Separate `dji1/dji2` model overrides:

```bash
./run.sh tmux_support_chain warehouse mode:=follow gui:=true \
  dji1_detector_backend:=onnx_cpu \
  dji1_detector_onnx_model:=/absolute/path/to/dji1_support_model.onnx \
  dji2_detector_backend:=onnx_cpu \
  dji2_detector_onnx_model:=/absolute/path/to/dji2_support_model.onnx
```

The current default model behavior is unchanged. This is only an explicit override path for Ruben/future detector work.

## RSSI / Signal Readiness

Full OMNeT/RSSI/signal-aware support positioning is not implemented in this phase.

The summary contract is prepared through relation fields:

```text
support_mux_relation_source
support_mux_relation_quality
support_mux_relation_note
```

Current meaning:

- `relation_source=odom` means the support UAV relation is based on fixed odom support slots around `dji0`.
- `relation_quality=not_evaluated` means no signal/RSSI quality has been computed yet.
- `relation_note=support_slots` identifies the current slot-based behavior.

Future values may include:

```text
rssi
signal
omnet
communication_quality
```

Future work can replace or augment fixed odom-slot support with RSSI/signal/communication-aware relation logic without redesigning the support observation summary contract.

## Run Commands

Baseline support-chain run:

```bash
./run.sh tmux_support_chain warehouse mode:=follow gui:=true
```

Support-chain with explicit relation placeholder:

```bash
./run.sh tmux_support_chain warehouse mode:=follow gui:=true \
  support_mux_relation_source:=odom
```

Support-chain with camera scanning enabled:

```bash
./run.sh tmux_support_chain warehouse mode:=follow gui:=true \
  support_mux_relation_source:=odom \
  support_camera_scan_enable:=true \
  support_bridge_gimbal:=true
```

## Topic Proof Checklist

Motion / spawn:

```bash
ros2 topic echo --once /dji1/pose
ros2 topic echo --once /dji2/pose
ros2 topic echo --once /dji1/pose_cmd
ros2 topic echo --once /dji2/pose_cmd
```

Support detection:

```bash
ros2 topic echo --once /coord/support/dji1/leader_detection_status
ros2 topic echo --once /coord/support/dji2/leader_detection_status
```

`dji0` summary:

```bash
ros2 topic echo --once /coord/dji0/support_observation_summary
```

UGV summary:

```bash
ros2 topic echo --once /coord/ugv/support_observation_summary
```

UGV awareness:

```bash
ros2 topic echo --once /coord/ugv/support_awareness_status
ros2 topic echo --once /coord/ugv/support_path_advisory
```

Camera scanner:

```bash
ros2 topic echo --once /coord/support/camera_scan_status
ros2 topic echo --once /dji1/update_pan
ros2 topic echo --once /dji2/update_pan
ros2 topic echo --once /dji1/update_tilt
ros2 topic echo --once /dji2/update_tilt
```

Camera info:

```bash
ros2 topic echo --once /dji1/camera0/camera_info
ros2 topic echo --once /dji2/camera0/camera_info
```

Image topics are large, so prefer `hz` or `rqt_image_view`:

```bash
ros2 topic hz /dji1/camera0/image_raw
ros2 topic hz /dji2/camera0/image_raw
```

`rqt_image_view` topics:

```text
/dji1/camera0/image_raw
/dji2/camera0/image_raw
```

## Thesis Evidence Checklist

Collect these before final writing if possible:

- Screenshot or short video of Gazebo showing the UGV, `dji0`, and `dji1/dji2` in support formation.
- Screenshot or short video of camera scanning with `/coord/support/camera_scan_status` active and pan/tilt commands publishing.
- Optional visual evidence of gimbal/camera movement if visible in Gazebo.
- `rqt_image_view` screenshots for `/dji1/camera0/image_raw` and `/dji2/camera0/image_raw`.
- Optional 30-60 second rosbag with key support-chain topics.

Suggested lightweight rosbag:

```bash
ros2 bag record \
  /dji1/pose \
  /dji2/pose \
  /dji1/pose_cmd \
  /dji2/pose_cmd \
  /coord/support/dji1/leader_detection_status \
  /coord/support/dji2/leader_detection_status \
  /coord/dji0/support_observation_summary \
  /coord/ugv/support_observation_summary \
  /coord/ugv/support_awareness_status \
  /coord/ugv/support_path_advisory \
  /coord/support/camera_scan_status \
  /dji1/update_pan \
  /dji2/update_pan \
  /dji1/update_tilt \
  /dji2/update_tilt \
  /dji1/camera0/camera_info \
  /dji2/camera0/camera_info
```

Optional image topics can be recorded too, but they are heavier:

```text
/dji1/camera0/image_raw
/dji2/camera0/image_raw
```

## Scope Boundary

Current thesis scope:

- Multi-UAV ROS 2 / Gazebo support-chain topology.
- `dji1/dji2` follow `dji0` using odom support slots.
- `dji1/dji2` publish support observation outputs.
- `dji0` aggregates/selects/summarizes support observations.
- The UGV receives structured support information.
- The UGV exposes a human-readable support awareness status.
- The UGV exposes a status-only support path advisory for future reaction/replanning integration.
- Optional support camera scanning supports image capture and validation.

Not claimed in this implementation:

- The UGV does not actively avoid obstacles based on support observations.
- The UGV does not actively replan from support observations.
- RSSI/OMNeT positioning is not implemented.
- Detector training/tuning is not part of this final support-chain phase.
- Autonomous UGV reaction based on support observations is future work.

Future work:

- RSSI/signal-aware support positioning.
- OMNeT communication integration.
- Active UGV reaction/replanning.
- Nav2/costmap integration using `/coord/ugv/support_awareness_status` or `/coord/ugv/support_path_advisory`.
- Detector retraining with support-camera data.
- Larger support-camera data collection and detector improvement.

## Final Status

Practical implementation status: complete enough.

Remaining work is evidence collection, screenshots/bags, and thesis writing. Further engineering should be framed as future work unless a concrete bug appears.
