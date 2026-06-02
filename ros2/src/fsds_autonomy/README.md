# fsds_autonomy

FSDS-first autonomy stack for mapping while driving, racing-line following, camera cone detection, obstacle safety, and later residual reinforcement learning.

## Run With The Simulator

Start the simulator and bridge first, then:

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch fsds_autonomy fsds_autonomy.launch.py
```

The live launch is deterministic mapping/racing. It does not start RL inside UE. On an unknown track it drives a centerline pass with a speed-aware bicycle/pure-pursuit steering preview until the mapper completes a long loop and sees the orange start/end gate again, progressively raises pre-loop speed as map quality and centerline coverage improve, caps that speed when visible-curve data is insufficient, avoids or stops for `/autonomy/obstacles`, builds/saves a map, and only uses the faster speed profile once the map is loop verified. If a usable closed-loop saved map already exists, the mapper keeps it as the saved/reference map and builds a separate clean live map from LiDAR-plus-stereo-confirmed cones; partial open-loop maps are ignored for race-from-map startup. Race-from-map live observations are not blindly written back, but every later pass can refine the saved map: matched same-color cones within `1.0 m` gently nudge saved cone positions, highly persistent new cones can be added, no saved cones are deleted, and quality/sanity checks plus history backups protect the previous map. It also publishes saved/reference and live blue/yellow boundary lines ordered by centerline progress, so the remembered cone boundaries remain available when nearby detections are sparse and far-apart cones are not connected across the map. Once a closed loop is confirmed and both boundary loops pass endpoint-gap/internal-step checks, Foxglove shades the inner island and an outer boundary band red and the reset monitor treats only the corridor between blue and yellow boundaries, with a `0.60 m` last-resort center safety tripwire from each side, as drivable. The controller also applies a boundary guard that nudges the steering target back toward the middle, slows the car, and brakes if the car drifts too close to either side. Racing lines account for the `1.00 m` vehicle collision width, `0.23 m` cone width, and a `1.35 m` safety/tracking margin; they are generated as smooth outside-apex-outside lines that use the available track width, then clamped inside the blue/yellow cone corridor before publishing/saving. Cone-hit/stuck/forbidden/offtrack reset locations are saved as `*.reset_events.json`, enriched with nearest line/cone context, published as X markers on `/autonomy/viz/reset_markers`, and fed back into future line generation so repeated reset clusters nudge the live/saved line away from the side where the car got stuck or hit a cone. Runtime map/controller/visible-corridor calculations use cones with confidence `>=0.50`; lower-confidence detections can still appear in debug overlays. Persistent map creation requires LiDAR plus `CameraStereo(cam1+cam2)` agreement by default, requires at least three observations over at least `0.20 s` before publishing/saving a landmark, prunes tentative landmarks that disappear for about `1.0 s`, ignores far/off-corridor cone points, and merges same-color landmarks within about `1.5 m`; raw LiDAR-only clusters and unmatched stereo points are visible for diagnostics but do not become permanent blue/yellow map cones. Overlapping camera detections that resolve to nearly the same cone position are de-duplicated before stereo/fusion/mapping. If the live racing line is sparse, low quality, or too close to live cones, it remains visible for debugging but the car drives the saved/reference line instead. Automatic reset is tied to simulator cone hits from `/fsds/testing_only/extra_info`, a sustained zero-speed stuck watchdog, and the loop-confirmed red forbidden area by default, not distance from the saved racing line. Live RL training is started separately after the map is closed; it begins as residual path/speed assistance and can graduate to full steering/throttle/brake while controller safety overrides remain last in the command chain.

Useful launch overrides:

```bash
ros2 launch fsds_autonomy fsds_autonomy.launch.py use_testing_odom:=true
ros2 launch fsds_autonomy fsds_autonomy.launch.py use_testing_odom:=false
ros2 launch fsds_autonomy fsds_autonomy.launch.py dataset_enabled:=true
ros2 launch fsds_autonomy fsds_autonomy.launch.py model_path:=/path/to/fine_tuned.pt
ros2 launch fsds_autonomy fsds_autonomy.launch.py control_enabled:=false auto_reset_enabled:=false
```

`use_testing_odom:=true` is only for bringup/debugging or validating maps already built in the simulator testing-odom frame. With `use_testing_odom:=false`, the state estimator uses GPS + GSS + IMU for `/autonomy/pose` and publishes `/autonomy/gps_pose` for inspection.
`control_enabled:=false auto_reset_enabled:=false` is the manual-driving preview mode: perception, mapping, behavior preview, and Foxglove camera overlays keep running, but the autonomy stack does not publish `/fsds/control_command` or reset the simulator. For simulator keyboard driving in this mode, start `fsds_ros2_bridge` with `manual_mode:=true`; API bridge mode still captures car control even when this autonomy launch is not commanding the car.

Start live RL only after the deterministic stack is running. The trainer waits for a closed map with usable blue/yellow boundaries, then rewards correct-direction sector progress and clean laps while penalizing cone hits, red-zone/offtrack resets, stuck resets, low clearance, backward progress, and unstable controls.

```bash
python3 src/fsds_autonomy/tools/train_rl_live.py \
  --algo SAC \
  --steps 200000 \
  --device auto \
  --out /home/hard/.fsds_autonomy/runs/rl_live_full_control
```

The curriculum starts with residual path/speed nudges, promotes to partial direct steering/throttle/brake after clean sector progress, and unlocks full direct controls after clean lap completion. TensorBoard logs are written under `/home/hard/.fsds_autonomy/runs`.

For faster live RL after a closed map exists, launch autonomy with the saved-map publisher and disable expensive mapping/visualization nodes:

```bash
ros2 launch fsds_autonomy fsds_autonomy.launch.py \
  map_dir:=/home/hard/.fsds_autonomy/maps \
  use_testing_odom:=true \
  camera_enabled:=false \
  mapper_enabled:=false \
  raceline_planner_enabled:=false \
  saved_map_publisher_enabled:=true \
  foxglove_visualizer_enabled:=false \
  drive_log_enabled:=false
```

This keeps the controller, safety resets, LiDAR fusion, behavior planner, and saved `/autonomy/racing_line` output, while avoiding repeated map/raceline generation during training.

Multiple live simulators require separate simulator settings files with unique `ApiServerPort` values and separate `ROS_DOMAIN_ID`s. The bridge accepts `api_port:=PORT` and `settings_path:=/path/to/settings.json`, so independent learners can run in parallel; sharing one SAC learner across multiple live simulators still requires a vectorized environment wrapper.

## Requirements

ROS-side requirements:

- ROS 2 Jazzy
- `cv_bridge`
- `foxglove_bridge`
- `nav_msgs`
- `sensor_msgs_py`
- `visualization_msgs`
- built FSDS packages: `fs_msgs`, `fsds_ros2_bridge`, `fsds_autonomy_msgs`, `fsds_autonomy`

Install/check on the GX10:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-cv-bridge \
  ros-jazzy-foxglove-bridge \
  ros-jazzy-nav-msgs \
  ros-jazzy-sensor-msgs-py \
  ros-jazzy-visualization-msgs
```

Optional ML/RL requirements:

- PyTorch/CUDA for YOLO and RL training
- Ultralytics for YOLO loading/training/export
- ONNX Runtime for exported detector tests
- Gymnasium and Stable-Baselines3 for live/full-control RL
- TensorBoard for RL/training metrics
- DepthAI for keeping the camera model path compatible with the RC/OAK-D stack

Install with:

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator
python3 -m pip install --user --break-system-packages \
  -r ros2/src/fsds_autonomy/requirements-ml.txt
```

Verify:

```bash
python3 -c 'import torch, ultralytics, onnxruntime, gymnasium, stable_baselines3, depthai; print(torch.cuda.is_available())'
```

## Main Runtime Topics

- Input: `/fsds/imu`, `/fsds/gss`, `/fsds/gps`, `/fsds/lidar/Lidar1`, `/fsds/lidar/Lidar2`, `/fsds/cam*/image_color`, `/fsds/signal/go`, `/fsds/testing_only/extra_info`
- Output: `/fsds/control_command`
- Debug: `/autonomy/gps_pose`, `/autonomy/lidar_cones`, `/autonomy/camera_cones`, `/autonomy/fused_cones`, `/autonomy/sensor_fusion_diagnostics`, `/autonomy/track_map`, `/autonomy/live_track_map`, `/autonomy/reference_track_map`, `/autonomy/racing_line`, `/autonomy/race_state`, `/autonomy/obstacles`, `/autonomy/path_offset`, `/autonomy/rl_path_offset`, `/autonomy/rl_speed_delta`, `/autonomy/rl_control_command`, `/autonomy/rl_direct_blend`, `/autonomy/offtrack_reset_status`
- Foxglove map: `/autonomy/viz/map_markers`, `/autonomy/viz/saved_map_markers`, `/autonomy/viz/live_map_markers`, `/autonomy/viz/reset_markers`, `/autonomy/viz/map_centerline_path`, `/autonomy/viz/optimal_racing_line_path`, `/autonomy/viz/current_drive_line_path`, `/autonomy/viz/live_blue_boundary_path`, `/autonomy/viz/live_yellow_boundary_path`, `/autonomy/viz/reference_blue_boundary_path`, `/autonomy/viz/reference_yellow_boundary_path`
- Foxglove camera debug: `/autonomy/viz/camera/cam1_debug`, `/autonomy/viz/camera/cam2_debug`, and `/autonomy/camera_debug` for the selected overlay camera; `/autonomy/viz/camera/cam1_overlay` and `/autonomy/viz/camera/cam2_overlay` remain debug aliases
- Foxglove camera run view: `/autonomy/viz/camera/cam1_run`, `/autonomy/viz/camera/cam2_run`, and `/autonomy/camera_run` for the selected overlay camera

In Foxglove, add a 3D panel with `fsds/map` as the fixed frame and enable `/autonomy/viz/map_markers` plus the path topics. The state estimator publishes `/tf` from `fsds/map` to `fsds/FSCar`, so map markers and the simulator sensor frames share one tree. Add `/autonomy/viz/saved_map_markers` when you want the remembered/reference cone positions and saved lines, and add `/autonomy/viz/live_map_markers` when you want the live/current map cone positions plus the recent fused cones the car currently sees. Add an Image panel for `/autonomy/viz/camera/cam1_debug` to see every camera detection, class confidence, stereo markers, behavior preview, and projected map lines. Add `/autonomy/viz/camera/cam1_run` when you only want the clean view of stereo detections and lines the car is actually using. Use `/autonomy/mapper_diagnostics` to watch tentative cone landmarks, confirmed cone landmarks, live map quality, and saved-map refinement version.

Obstacle avoidance is intentionally conservative: the behavior planner tries a temporary left/right `/autonomy/path_offset` only when LiDAR obstacle clearance is good enough. Otherwise it slows or triggers emergency brake before impact.

The controller publishes `/fsds/control_command` at `100 Hz` and uses a speed-aware pure-pursuit/bicycle steering model: it keeps a progress cursor on the selected drive line, advances that cursor to a future point, computes the curvature needed to reach it, converts that to a front-wheel angle using `wheelbase_m`, and rate-limits the normalized FSDS steering command. The cursor prevents the future target from jumping backward or sideways when the live map changes between frames, and a small lateral target filter smooths the early live-cone fallback before the centerline is stable. If a live-learning map closes but has not been saved and loaded as `race_from_map`, the controller keeps using centerline/edge-recovery safety and the behavior planner stays in `LoopVerifyCenterline` instead of immediately trusting the generated racing line. It also uses a short `launch_throttle` boost while the car is almost stopped, then returns to proportional speed control once `/autonomy/speed` rises above the launch threshold. Live RL can publish residual line/speed topics plus `/autonomy/rl_control_command` and `/autonomy/rl_direct_blend`; stale RL messages are ignored, and emergency/no-go, close-cone, boundary, and zero-target safety overrides are applied after RL blending.

Planning geometry comes from `/autonomy/fused_cones`: LiDAR cone clusters are matched with `CameraStereo(cam1+cam2)` cones, and the fused cone position blends both measurements when they agree. Persistent mapping accepts only LiDAR-plus-stereo-confirmed cones by default; unmatched stereo cones can still appear in diagnostics/overlays, and monocular camera boxes are kept for overlays and color matching unless `include_camera_only_geometry` is explicitly enabled.

Use `/autonomy/sensor_fusion_diagnostics` to inspect LiDAR-vs-stereo agreement. Each row reports the nearest LiDAR cone, stereo cone, X/Y/range/bearing deltas, match/fusion status, and OK/WARN/ERROR thresholds.

Cone side convention is fixed to the FSDS track layout: blue cones are the left boundary, yellow cones are the right boundary, and orange/large-orange cones are start/end gate markers. The mapper keeps orange cones for visualization and saved maps, Foxglove labels them as `START/END`, and the centerline/racing-line planner only pairs blue/yellow boundary cones.

## Model Workflow

Install optional ML packages:

```bash
python3 -m pip install --user --break-system-packages \
  -r ros2/src/fsds_autonomy/requirements-ml.txt
```

Record synthetic FSDS labels:

```bash
ros2 launch fsds_autonomy fsds_autonomy.launch.py dataset_enabled:=true use_testing_odom:=true
```

Train:

```bash
python3 ros2/src/fsds_autonomy/tools/train_yolo.py \
  --data ~/.fsds_autonomy/datasets/fsds_cones/fsds_cones.yaml \
  --model /home/hard/Desktop/Driverless_FSD_HARD/ros2_ws/src/cone_detection_model/yolo26n.pt
```

The first implementation intentionally keeps deterministic LiDAR/planning/controller safety in charge. YOLO improves color and unknown-object recognition, but does not replace safety logic.
