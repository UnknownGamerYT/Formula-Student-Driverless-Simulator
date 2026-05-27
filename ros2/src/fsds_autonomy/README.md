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

The live launch is deterministic mapping/racing. It does not start RL inside UE. On an unknown track it drives a slow first pass, estimates visible straight/curve speed from `/autonomy/fused_cones`, avoids or stops for `/autonomy/obstacles`, builds/saves a map, and only uses the faster speed profile once map quality is high enough. Residual RL training is run separately from a saved map.

Useful launch overrides:

```bash
ros2 launch fsds_autonomy fsds_autonomy.launch.py use_testing_odom:=true
ros2 launch fsds_autonomy fsds_autonomy.launch.py dataset_enabled:=true
ros2 launch fsds_autonomy fsds_autonomy.launch.py model_path:=/path/to/fine_tuned.pt
ros2 launch fsds_autonomy fsds_autonomy.launch.py control_enabled:=false auto_reset_enabled:=false
```

`use_testing_odom:=true` is only for bringup/debugging. Runtime mapping and driving should not depend on testing-only topics.
`control_enabled:=false auto_reset_enabled:=false` is the manual-driving preview mode: perception, mapping, behavior preview, and Foxglove camera overlays keep running, but the autonomy stack does not publish `/fsds/control_command` or reset the simulator. For simulator keyboard driving in this mode, start `fsds_ros2_bridge` with `manual_mode:=true`; API bridge mode still captures car control even when this autonomy launch is not commanding the car.

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
- Gymnasium and Stable-Baselines3 for residual RL
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

- Input: `/fsds/imu`, `/fsds/gss`, `/fsds/gps`, `/fsds/lidar/Lidar1`, `/fsds/lidar/Lidar2`, `/fsds/cam*/image_color`, `/fsds/signal/go`
- Output: `/fsds/control_command`
- Debug: `/autonomy/lidar_cones`, `/autonomy/camera_cones`, `/autonomy/fused_cones`, `/autonomy/sensor_fusion_diagnostics`, `/autonomy/track_map`, `/autonomy/racing_line`, `/autonomy/race_state`, `/autonomy/obstacles`, `/autonomy/path_offset`
- Foxglove map: `/autonomy/viz/map_markers`, `/autonomy/viz/map_centerline_path`, `/autonomy/viz/optimal_racing_line_path`, `/autonomy/viz/current_drive_line_path`
- Foxglove camera: `/autonomy/viz/camera/cam1_overlay`, `/autonomy/viz/camera/cam2_overlay`, and `/autonomy/camera_debug` for the selected overlay camera

In Foxglove, add a 3D panel with `fsds/map` as the fixed frame and enable `/autonomy/viz/map_markers` plus the three path topics. Add an Image panel for `/autonomy/viz/camera/cam1_overlay` to see camera detections, class confidence, stereo-triangulated cone markers, behavior preview, the current drive line, and the optimal racing line projected into the camera view.

Obstacle avoidance is intentionally conservative: the behavior planner tries a temporary left/right `/autonomy/path_offset` only when LiDAR obstacle clearance is good enough. Otherwise it slows or triggers emergency brake before impact.

The controller uses a short `launch_throttle` boost while the car is almost stopped, then returns to proportional speed control once `/autonomy/speed` rises above the launch threshold.

Planning geometry comes from `/autonomy/fused_cones`: LiDAR cone clusters are matched with `CameraStereo(cam1+cam2)` cones, and the fused cone position blends both measurements when they agree. Unmatched stereo cones can still enter the map/path pipeline; monocular camera boxes are kept for overlays and color matching unless `include_camera_only_geometry` is explicitly enabled.

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
