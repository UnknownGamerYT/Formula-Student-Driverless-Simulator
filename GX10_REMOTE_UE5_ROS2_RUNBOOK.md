# GX10 Remote UE5 + ROS 2 Daily Runbook

Use this guide when the GX10 server is already set up and you only need to start the remote desktop, launch the UE5 Chaos FSDS simulator, run the ROS 2 Jazzy bridge, and test control from your own PC.

For laptop/PC requirements, see `GX10_REMOTE_PC_REQUIREMENTS.md`.

Validated on 2026-05-26:

- GX10 host: `gx10-d190`
- GX10 user: `hard`
- GX10 remote IP used during testing: `100.81.202.64`
- Normal logged-in desktop display: `:1`
- GDM login-screen display: `:0`
- Simulator package: `UE5Builds/LinuxArm64`
- Simulator RPC port: `41451`
- Browser noVNC port: `6080`
- VNC backend port: `5900`
- VNC password: same as the PC/Linux password for user `hard`

## What Runs Where

Use separate terminals so long-running processes can stay open.

| Location | Purpose | Command family |
| --- | --- | --- |
| Your PC terminal 1 | SSH into GX10 and start `x11vnc` | `ssh ...`, then `x11vnc ...` |
| Your PC terminal 2 | SSH into GX10 and start noVNC proxy | `ssh ...`, then `websockify ...` |
| Your PC terminal 3 | Browser tunnel from your PC to GX10 | `ssh -N -L ...` |
| Your PC browser | Remote GX10 desktop | `http://localhost:6080/...` |
| Your PC terminal 4 | SSH into GX10 and start UE5 simulator | `ssh ...`, then `Blocks.sh ...` |
| Your PC terminal 5 | SSH into GX10 and run ROS bridge/tests | `ssh ...`, then `ros2 ...` |

It is normal for `x11vnc`, `websockify`, `ros2 launch`, `Blocks.sh`, and `ssh -N -L ...` to keep the terminal busy and not return to a prompt.

## 0. One-Time Requirements Check

Skip this section during normal daily use if the GX10 is already set up. Use it when onboarding a new machine, after OS changes, or if commands in the runbook are missing.

### PC Requirements

Your own PC only needs:

- SSH client
- modern browser for noVNC
- optional Foxglove Studio
- optional browser access to TensorBoard

Details are in `GX10_REMOTE_PC_REQUIREMENTS.md`.

### GX10 System Packages

On the GX10:

```bash
sudo apt update
sudo apt install -y \
  x11vnc \
  novnc \
  websockify \
  openssh-server \
  netcat-openbsd \
  mesa-utils \
  vulkan-tools \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep
```

### GX10 ROS 2 Packages

The autonomy stack expects ROS 2 Jazzy plus these packages:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-cv-bridge \
  ros-jazzy-foxglove-bridge \
  ros-jazzy-geometry-msgs \
  ros-jazzy-image-transport \
  ros-jazzy-nav-msgs \
  ros-jazzy-rclpy \
  ros-jazzy-rosgraph-msgs \
  ros-jazzy-sensor-msgs \
  ros-jazzy-sensor-msgs-py \
  ros-jazzy-std-msgs \
  ros-jazzy-tf2-ros \
  ros-jazzy-visualization-msgs
```

Confirm ROS and colcon:

```bash
source /opt/ros/jazzy/setup.bash
ros2 --version
colcon version-check || true
```

### Python ML/RL Packages

Install the optional ML/RL stack used by YOLO, ONNX export/inference, DepthAI compatibility, TensorBoard, and residual RL:

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator
python3 -m pip install --user --break-system-packages \
  -r ros2/src/fsds_autonomy/requirements-ml.txt
```

`requirements-ml.txt` pins `numpy<2.0` and `opencv-python<4.12` to avoid breaking ROS Jazzy/cv_bridge compatibility.

Verify Python dependencies:

```bash
source /opt/ros/jazzy/setup.bash
source /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2/install/setup.bash 2>/dev/null || true

python3 -c 'import rclpy, cv_bridge, cv2, torch, ultralytics, onnxruntime, gymnasium, stable_baselines3, depthai; print("Python deps OK"); print("CUDA:", torch.cuda.is_available(), torch.cuda.get_device_name(0) if torch.cuda.is_available() else "none")'
```

Expected on the GX10:

```text
Python deps OK
CUDA: True NVIDIA GB10
```

### Model Weights

The autonomy launch defaults to the RC repo's YOLO starting weight:

```text
/home/hard/Desktop/Driverless_FSD_HARD/ros2_ws/src/cone_detection_model/yolo26n.pt
```

Check it exists:

```bash
ls -lh /home/hard/Desktop/Driverless_FSD_HARD/ros2_ws/src/cone_detection_model/yolo26n.pt
```

Check whether it is actually cone-trained:

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 src/fsds_autonomy/tools/check_yolo_model.py \
  /home/hard/Desktop/Driverless_FSD_HARD/ros2_ws/src/cone_detection_model/yolo26n.pt
```

If this prints `NOT_A_CONE_MODEL`, the file is only a generic pretrained model. That is still useful as a training starting point, but live camera cone boxes will come from the low-trust HSV fallback until you fine-tune and launch with a cone model.

If it is missing, download/cache models after installing ML dependencies:

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator
python3 ros2/src/fsds_autonomy/tools/download_models.py \
  --out /home/hard/.fsds_autonomy/models/pretrained
```

Then launch with:

```bash
ros2 launch fsds_autonomy fsds_autonomy.launch.py \
  model_path:=/home/hard/.fsds_autonomy/models/pretrained/yolo26n.pt
```

### Build The ROS Workspace

Whenever packages or messages are added/changed:

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select fs_msgs fsds_ros2_bridge fsds_autonomy_msgs fsds_autonomy --symlink-install
source install/setup.bash
```

Quick autonomy launch check:

```bash
ros2 launch fsds_autonomy fsds_autonomy.launch.py --show-args
```

You should see `control_enabled`, `auto_reset_enabled`, `model_path`, `map_dir`, and `dataset_enabled` in the launch arguments.

## 1. Start The VNC Backend

On your PC, open terminal 1:

```bash
ssh hard@100.81.202.64
```

After login, this terminal is running on the GX10.

Check which display exists:

```bash
ls -la /tmp/.X11-unix
loginctl list-sessions
```

Choose the correct path below.

If you want a clean viewer start, stop any old noVNC and VNC backends before starting Path A or Path B:

```bash
pgrep -f '[w]ebsockify.*6080' | xargs -r kill
pgrep -f '[x]11vnc.*5900' | xargs -r kill
sleep 2
pgrep -f '[w]ebsockify.*6080' | xargs -r kill -9
pgrep -f '[x]11vnc.*5900' | xargs -r kill -9
ss -ltnp | grep -E ':(5900|6080)' || true
```

No output for ports `5900` and `6080` means both viewer services are closed.

### Path A: The `hard` Desktop Is Already Active

Use this if `X1` exists and `loginctl` shows a local `hard` session. This is the normal path after the GX10 user is already logged in.

On the GX10 SSH terminal:

```bash
x11vnc \
  -display :1 \
  -auth guess \
  -rfbauth ~/.vnc/passwd \
  -localhost \
  -forever \
  -shared \
  -noxdamage \
  -repeat \
  -rfbport 5900
```

Leave this terminal running.

Normal output:

```text
Using X display :1
Listening for VNC connections on TCP port 5900
The VNC desktop is: localhost:0
```

If the active `hard` display is `X0` instead of `X1`, use `-display :0` here and use `DISPLAY=:0` in later display commands.

### Path B: The GX10 Is At The Ubuntu Login Screen

Use this if `x11vnc -display :1 -auth guess ...` fails, or if only `X0` exists and `loginctl` shows the local session owner as `gdm`.

On the GX10 SSH terminal:

```bash
sudo x11vnc \
  -display :0 \
  -auth /run/user/126/gdm/Xauthority \
  -rfbauth /home/hard/.vnc/passwd \
  -localhost \
  -forever \
  -shared \
  -noshm \
  -noxdamage \
  -nowf \
  -noscr \
  -rfbport 5900
```

Leave this terminal running until you log in through the browser.

Normal output:

```text
Using X display :0
Listening for VNC connections on TCP port 5900
The VNC desktop is: localhost:0
```

The `-noshm` flag is required for the GDM login screen. Without it, `x11vnc` can fail with:

```text
X11 MIT Shared Memory Attach failed
BadAccess (attempt to access private resource denied)
```

## 2. Start The noVNC Browser Proxy

On your PC, open terminal 2:

```bash
ssh hard@100.81.202.64
```

After login, start the proxy on the GX10:

```bash
websockify --web=/usr/share/novnc/ 127.0.0.1:6080 127.0.0.1:5900
```

Leave this terminal running.

Normal output:

```text
WebSocket server settings:
  - Listen on 127.0.0.1:6080
  - proxying from 127.0.0.1:6080 to 127.0.0.1:5900
```

If `websockify` prints `404 File not found`, that usually means the browser opened `http://localhost:6080/` without `/vnc.html`. Use the full URL in step 4.

## 3. Open The Browser Tunnel From Your PC

On your PC, open terminal 3:

```bash
ssh -N -L 6080:localhost:6080 hard@100.81.202.64
```

After login this command intentionally appears to hang. That means the tunnel is open. Leave it running.

## 4. Open The GX10 Desktop In Your Browser

On your PC, open:

```text
http://localhost:6080/vnc.html?host=localhost&port=6080
```

Click connect and enter the VNC password. On this GX10, the VNC password is the same as the PC/Linux password for user `hard`.

If you see the Ubuntu login screen, log in as `hard`.

## 5. If You Logged In From The Ubuntu Login Screen

If you used Path B, logging in usually closes the GDM login-screen X server and creates the real `hard` desktop on `:1`.

When that happens, this is normal in terminal 1:

```text
caught XIO error
```

This is normal in terminal 2:

```text
Failed to connect to 127.0.0.1:5900: [Errno 111] Connection refused
```

Go back to terminal 1, which is still SSHed into the GX10, and check the display:

```bash
ls -la /tmp/.X11-unix
loginctl list-sessions
```

If `X1` now exists and belongs to `hard`, restart `x11vnc` on the user desktop:

```bash
x11vnc \
  -display :1 \
  -auth guess \
  -rfbauth ~/.vnc/passwd \
  -localhost \
  -forever \
  -shared \
  -rfbport 5900
```

Keep `websockify` in terminal 2 and the SSH tunnel in terminal 3 running. Refresh the browser page and connect again.

If the GX10 desktop is locked later, unlock it in the browser with the normal Ubuntu password for `hard`.

## 6. Confirm Desktop And GPU

On any GX10 SSH terminal:

```bash
DISPLAY=:1 xset dpms force on
DISPLAY=:1 xrandr --query
DISPLAY=:1 glxinfo -B
DISPLAY=:1 vulkaninfo --summary
```

Expected:

- An active display is listed, usually `HDMI-0`.
- OpenGL renderer is `NVIDIA GB10`.
- Vulkan lists `NVIDIA GB10`.

If the logged-in `hard` desktop is on `:0`, use `DISPLAY=:0` instead.

## 7. Start The UE5 Visual Simulator

On your PC, open terminal 4:

```bash
ssh hard@100.81.202.64
```

On the GX10:

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/UE5Builds/LinuxArm64
```

Start the simulator with rendering enabled:

```bash
DISPLAY=:1 ./Blocks.sh \
  -vulkan \
  -windowed \
  -ResX=1280 \
  -ResY=720 \
  -nosound \
  -NoVSync \
  -stdout \
  -FullStdOutLogOutput \
  -log \
  -ExecCmds="r.VSync 0,t.MaxFPS 0,stat fps,stat unit,stat gpu"
```

Do not use `-nullrhi` for a visual run. `-nullrhi` disables rendering.

The simulator window should appear in the browser desktop.

## 8. Confirm The Simulator RPC Port

On any GX10 SSH terminal:

```bash
nc -vz 127.0.0.1 41451
```

Expected:

```text
Connection to 127.0.0.1 41451 port [tcp/*] succeeded
```

## 9. Start The ROS 2 Jazzy Bridge

On your PC, open terminal 5:

```bash
ssh hard@100.81.202.64
```

On the GX10:

```bash
source /opt/ros/jazzy/setup.bash
source /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2/install/setup.bash

ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py host:=127.0.0.1 manual_mode:=false timeout:=120.0
```

Expected log lines:

```text
Connected to the simulator!
AirsimROSWrapper Initialized!
```

This is the autonomous/API-control bridge mode. Use it when terminal 6 will run the autonomy controller and publish `/fsds/control_command`. In this mode the simulator keyboard vehicle controls are disabled because ROS has control of the car.

For manual keyboard driving, stop the bridge above and start it with `manual_mode:=true` instead:

```bash
source /opt/ros/jazzy/setup.bash
source /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2/install/setup.bash

ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py host:=127.0.0.1 manual_mode:=true timeout:=120.0
```

Use only one bridge mode at a time. `settings.json` already has `RemoteControlID: -1`, which is the keyboard-control setting for this FSDS build.

When driving manually through noVNC, click inside the Unreal window first so it has keyboard focus. Vehicle controls are arrow keys, Space for foot brake, and End for handbrake. Camera controls are `M` for manual camera, arrow keys to move the camera, PageUp/PageDown for vertical movement, `W/S` pitch, `A/D` yaw, `Q/E` roll, `/` chase view, and `F` FPV.

## 10. Verify ROS Topics

Open another GX10 SSH terminal if needed:

```bash
source /opt/ros/jazzy/setup.bash
source /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2/install/setup.bash

ros2 topic list
```

Expected topics include:

```text
/clock
/fsds/cam1/image_color
/fsds/cam2/image_color
/fsds/control_command
/fsds/gps
/fsds/gss
/fsds/imu
/fsds/lidar/Lidar1
/fsds/lidar/Lidar2
/fsds/testing_only/odom
/fsds/wheel_states
```

Check live rates:

```bash
ros2 topic hz /clock
ros2 topic hz /fsds/testing_only/odom
ros2 topic hz /fsds/imu
ros2 topic hz /fsds/gps
ros2 topic hz /fsds/lidar/Lidar1
ros2 topic hz /fsds/cam1/image_color
```

Observed reference rates during validation:

```text
/clock: about 100 Hz
/fsds/testing_only/odom: about 95-105 Hz
/fsds/imu: about 249 Hz
/fsds/gps: about 10 Hz
/fsds/lidar/Lidar1: about 10 Hz
/fsds/cam1/image_color: about 5 Hz
```

## 11. Start The Autonomy Stack

Use this path when you want the car to drive, map the track, publish Foxglove visualizations, and run the hard-coded off-track reset monitor.

This launch is deterministic live driving, not live RL. The first pass is intentionally conservative: it starts around `1.6 m/s`, ramps slowly, speeds up when the visible cone corridor looks straight, slows for visible curves, avoids obstacles if there is room, stops if the corridor is blocked, and only uses the generated speed profile after a saved/loaded map reaches good quality. RL training is a separate residual learner that starts after a map exists.

On your PC, open terminal 6:

```bash
ssh hard@100.81.202.64
```

On the GX10:

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

colcon build --packages-select fsds_autonomy_msgs fsds_autonomy --symlink-install

ros2 launch fsds_autonomy fsds_autonomy.launch.py \
  map_dir:=/home/hard/.fsds_autonomy/maps \
  dataset_dir:=/home/hard/.fsds_autonomy/datasets/fsds_cones \
  auto_reset_enabled:=true \
  dataset_enabled:=false
```

Leave this terminal running. This starts:

- LiDAR cone detection
- camera cone detection with YOLO, falling back to color detection if needed
- dual-camera cone triangulation from `cam1` and `cam2`
- sensor fusion, blending LiDAR cone clusters with matching `CameraStereo(cam1+cam2)` cone positions and using monocular camera boxes mainly for color/overlay
- map building
- racing-line planning
- behavior planning
- controller publishing to `/fsds/control_command`
- Foxglove visualization topics
- hard-coded off-track reset monitor

For manual driving with autonomy perception/preview only, use the same launch but turn off control and auto reset:

```bash
ros2 launch fsds_autonomy fsds_autonomy.launch.py \
  map_dir:=/home/hard/.fsds_autonomy/maps \
  dataset_dir:=/home/hard/.fsds_autonomy/datasets/fsds_cones \
  control_enabled:=false \
  auto_reset_enabled:=false \
  dataset_enabled:=false
```

In this preview mode the stack still publishes camera detections, stereo-triangulated cone positions, confidence overlays, map/racing-line projection, behavior state, target speed, and Foxglove visualization topics, but it does not publish `/fsds/control_command` and it does not reset the simulator. This is the mode to use when you want to manually drive around and watch what the autonomy stack would do.

Manual preview also needs the ROS bridge from step 9 to be running with `manual_mode:=true`. If the bridge is still running with `manual_mode:=false`, the autonomy stack may be off but the bridge still holds API control, so keyboard driving will not move the car.

First-lap speed tuning lives in:

```text
/home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2/src/fsds_autonomy/config/autonomy.yaml
```

The most useful knobs are:

```yaml
fsds_behavior_planner:
  ros__parameters:
    first_lap_speed_mps: 1.6
    mapping_speed_mps: 2.2
    race_speed_mps: 5.0
    startup_ramp_sec: 10.0
    target_accel_limit_mps2: 0.40
    map_quality_for_speed_profile: 0.65
    visible_curve_lookahead_m: 18.0
    visible_curve_min_speed_mps: 1.1
    visible_straight_speed_mps: 3.0
    visible_curve_straight_angle_rad: 0.08
    visible_curve_hard_angle_rad: 0.55
    obstacle_avoid_enabled: true
    obstacle_avoid_distance_m: 10.0
    obstacle_avoid_offset_m: 0.85
    obstacle_avoid_clearance_m: 0.55
    obstacle_avoid_speed_mps: 1.4
    obstacle_avoid_hold_sec: 1.0
    obstacle_stop_if_blocked_distance_m: 5.0

fsds_camera_detector:
  ros__parameters:
    stereo_enabled: true
    stereo_max_time_delta_sec: 0.30
    stereo_min_confidence: 0.30
    stereo_min_disparity_rad: 0.006

fsds_sensor_fusion:
  ros__parameters:
    include_camera_only_geometry: false
    include_stereo_camera_geometry: true
    stereo_camera_min_confidence: 0.45
    stereo_position_gate_m: 2.0
    stereo_lidar_position_blend: 0.35
    diagnostics_warn_distance_m: 0.75
    diagnostics_error_distance_m: 1.50

fsds_controller:
  ros__parameters:
    throttle_kp: 0.12
    max_throttle: 0.55
    max_path_offset_m: 1.0
    launch_throttle: 0.45
    launch_speed_threshold_mps: 0.60
```

The visible-curve logic reads `/autonomy/fused_cones`, bins cones ahead of the car into a local center corridor, estimates the bend angle, and publishes the final target on `/autonomy/target_speed`. Watch `/autonomy/race_state` to see statuses such as `visible_curve speed=2.45mps angle=12.2deg samples=4`.

The obstacle logic reads `/autonomy/obstacles`. If an object is close and centered, it checks left/right clearance, publishes `/autonomy/path_offset` for a gentle dodge, caps speed to `obstacle_avoid_speed_mps`, and switches to emergency braking if the object is too close or no side has enough clearance. Watch for statuses such as `avoid_obstacle side=left ...`, `blocked_obstacle_slow ...`, or `blocked_obstacle_stop ...`.

The launch throttle exists because the FSDS vehicle can need more than the normal proportional throttle to break static friction from rest. It only applies while target speed is positive and measured speed is below `launch_speed_threshold_mps`; normal speed control takes back over once the car is rolling.

Cone geometry for mapping and path planning comes from `/autonomy/fused_cones`. When LiDAR and stereo camera detections match, the fused position is a weighted LiDAR/stereo blend. Unmatched `CameraStereo(cam1+cam2)` cones can still be used; camera-only monocular cone positions are intentionally excluded by default because single-box depth estimates are too noisy for steering. Camera detections still feed Foxglove overlays and can color matched LiDAR cones.

To inspect whether stereo camera geometry agrees with LiDAR:

```bash
ros2 topic echo /autonomy/sensor_fusion_diagnostics std_msgs/msg/String
```

The first line gives the overall state and counts. Each following row compares one stereo cone with the nearest LiDAR cone, including `dist_error`, `dx`, `dy`, range error, bearing error, whether it was matched into a fused cone, and whether it crossed the WARN/ERROR thresholds.

Expected useful log lines:

```text
YOLO detector loaded for camera cones
Dataset recorder disabled; set enabled:=true to save synthetic labels
```

The autonomy stack waits for `/fsds/signal/go`. The bridge publishes this signal, so the car should begin driving after the bridge and autonomy stack are both running.

### Automatic Off-Track Reset

The reset monitor watches `/autonomy/pose` against `/autonomy/racing_line`. If the car stays farther than `4.5 m` from the generated centerline/racing line for `1.5 s`, it publishes full brake and calls:

```text
/fsds/reset
```

Watch the reset state:

```bash
ros2 topic echo /autonomy/offtrack_reset_status std_msgs/msg/String
```

Manual reset switch:

```bash
ros2 service call /fsds/reset fs_msgs/srv/Reset "{wait_on_last_task: false}"
```

Turn automatic reset off for debugging:

```bash
ros2 launch fsds_autonomy fsds_autonomy.launch.py auto_reset_enabled:=false
```

If the car resets too easily or not aggressively enough, edit:

```text
/home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2/src/fsds_autonomy/config/autonomy.yaml
```

Tune these values:

```yaml
fsds_offtrack_reset_monitor:
  ros__parameters:
    offtrack_distance_m: 4.5
    offtrack_hold_sec: 1.5
    reset_cooldown_sec: 8.0
```

## 12. Run RL Learning From A Saved Map

The current RL learner is a lightweight residual-training loop built from saved simulator maps. It does not directly drive the live UE car yet. The live car drives with deterministic mapping/racing behavior; once a map exists, RL can train speed/path-offset behavior from that saved map.

First let the autonomy stack create a map. Saved maps are written here:

```text
/home/hard/.fsds_autonomy/maps/<track_id>.json
```

Check what maps exist:

```bash
ls -lh /home/hard/.fsds_autonomy/maps
```

Start residual RL training in another GX10 terminal:

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 src/fsds_autonomy/tools/train_rl_residual.py \
  --map-dir /home/hard/.fsds_autonomy/maps \
  --track-id A \
  --algo SAC \
  --steps 200000 \
  --out /home/hard/.fsds_autonomy/runs/rl_residual
```

Use the actual track id from the GO signal and map filename. If the bridge is launched with `track_name:=A`, the default track id is `A`.

The Gymnasium RL environment resets itself automatically when the residual policy goes off track. This is separate from the live simulator reset monitor, which calls `/fsds/reset` for the UE car.

Open TensorBoard for RL metrics:

```bash
tensorboard --logdir /home/hard/.fsds_autonomy/runs/rl_residual --host 0.0.0.0 --port 6006
```

From your PC:

```bash
ssh -N -L 6006:localhost:6006 hard@100.81.202.64
```

Open:

```text
http://localhost:6006
```

## 13. Optional Manual Move And Stop Test

Publish throttle:

```bash
source /opt/ros/jazzy/setup.bash
source /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2/install/setup.bash

ros2 topic pub -r 20 /fsds/control_command fs_msgs/msg/ControlCommand \
  "{throttle: 0.4, steering: 0.0, brake: 0.0}"
```

The car should move in the browser desktop.

Stop the car:

```bash
ros2 topic pub --once /fsds/control_command fs_msgs/msg/ControlCommand \
  "{throttle: 0.0, steering: 0.0, brake: 1.0}"
```

Then stop the throttle publisher with `Ctrl+C`.

## 14. Open And Visualize In Foxglove

On the GX10:

```bash
source /opt/ros/jazzy/setup.bash
source /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2/install/setup.bash
ros2 run foxglove_bridge foxglove_bridge
```

From your PC, open another terminal:

```bash
ssh -N -L 8765:localhost:8765 hard@100.81.202.64
```

Open Foxglove Studio and connect to:

```text
ws://localhost:8765
```

Suggested Foxglove layout:

- 3D panel with fixed frame `fsds/map`
- Image panel for `/autonomy/viz/camera/cam1_overlay`
- Raw Messages panel for `/autonomy/race_state`
- Raw Messages panel for `/autonomy/offtrack_reset_status`
- Raw Messages panel for `/autonomy/sensor_fusion_diagnostics`
- Plot panel for `/autonomy/path_offset`
- Plot panel for `/autonomy/speed` and `/autonomy/target_speed`

Enable these 3D topics:

```text
/autonomy/viz/map_markers
/autonomy/viz/map_centerline_path
/autonomy/viz/optimal_racing_line_path
/autonomy/viz/current_drive_line_path
/fsds/lidar/Lidar1
/fsds/lidar/Lidar2
```

Useful camera topics:

```text
/fsds/cam1/image_color
/fsds/cam2/image_color
/autonomy/viz/camera/cam1_overlay
/autonomy/viz/camera/cam2_overlay
/autonomy/camera_debug
```

What to look for:

- Blue markers are the left boundary, yellow markers are the right boundary, and orange/large-orange markers are the start/end gate labeled `START/END`.
- Orange markers are saved and visualized, but the centerline/racing-line planner ignores them as normal boundary cones.
- Transparent smaller markers are live fused cone detections.
- Cyan line is the generated center/current drive line.
- Magenta line is the optimal racing line.
- White arrow is the car pose.
- Red spheres are live obstacle clusters.
- Bright green line is the temporary obstacle-avoidance offset.
- The camera overlay shows detection boxes, class confidence, stereo confidence markers, behavior preview, target speed/path offset, and projected drive/racing lines.

If the camera overlay boxes appear but the projected lines look shifted, the camera extrinsics/FOV in `settings.json` and `autonomy.yaml` need calibration.

## 15. Vehicle Physics Reality Check

The default FSDS vehicle dynamics are not controlled from the ROS autonomy config. `settings.json` can change spawn, sensors, camera/lidar layout, clock speed, collision settings, and which vehicle pawn is used, but the tire/friction/suspension/engine behavior is inside the Unreal PhysX vehicle pawn assets.

Current easy runtime facts:

- The active car pawn is `TechnionCarPawn` from `settings.json`.
- The FSDS docs list the competition vehicle as about `255 kg`, `~27 m/s` max speed, and `0.3` drag coefficient.
- Wheel defaults in the AirSim car source use `40 deg` front steer angle and PhysX tire configs stored as `.uasset` files.

Recommended order before editing Unreal physics:

1. Tune the autonomy stack first: `first_lap_speed_mps`, `mapping_speed_mps`, `startup_ramp_sec`, `target_accel_limit_mps2`, `throttle_kp`, and `max_throttle`.
2. Use Foxglove plots for `/autonomy/speed`, `/autonomy/target_speed`, and `/fsds/control_command` to see whether the controller is commanding too much or the vehicle model is sliding despite gentle commands.
3. If the vehicle still feels unrealistic with gentle throttle and braking, edit a copied vehicle pawn in the Unreal project under `UE4Project/Plugins/AirSim/Content/VehicleAdv/`, then rebuild/package the simulator.

Physics-side knobs to review in Unreal:

- tire friction configs: `Vehicle_FrontTireConfig`, `Vehicle_BackTireConfig`
- wheel assets: radius, width, damping, mass, max steer angle
- vehicle movement component: mass, center of gravity, drag, torque curve, differential, brake torque, steering curve
- physics project settings: substepping and friction combine mode

Keep ROS-side safety limits conservative even after physics tuning; the simulator runtime can still bottleneck or jitter under heavy rendering, bridge, YOLO, and Foxglove load.

## 16. Performance Checks

GPU:

```bash
nvidia-smi dmon -s pucm -d 1
```

UE and ROS CPU/RAM:

```bash
pidstat -u -r -p $(pgrep -d, -f 'FSOnline/Binaries/Linux/Blocks|fsds_ros2_bridge|x11vnc|websockify') 1
```

ROS bandwidth:

```bash
source /opt/ros/jazzy/setup.bash
source /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2/install/setup.bash

ros2 topic bw /fsds/cam1/image_color
ros2 topic bw /fsds/lidar/Lidar1
ros2 topic hz /autonomy/viz/camera/cam1_overlay
ros2 topic hz /autonomy/viz/map_markers
```

Observed reference values during validation:

```text
UE visual FPS: about 60 FPS at 1280x720 windowed on a 60 Hz desktop
GPU shader load: about 65-76%
UE Blocks CPU: about 2.6-2.8 CPU cores
x11vnc/websockify CPU: low, usually under 10% combined
/fsds/cam1/image_color bandwidth: about 9-11 MB/s
/fsds/lidar/Lidar1 bandwidth: about 123 KB/s
```

noVNC is for setup and inspection. It is not a high-FPS game streaming protocol. If the browser view feels choppy while the UE overlay shows about 60 FPS, the simulator is probably fine and the remote viewing path is the limiting part.

## 17. Stop Everything

Send brake once:

```bash
source /opt/ros/jazzy/setup.bash
source /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2/install/setup.bash

ros2 topic pub --once /fsds/control_command fs_msgs/msg/ControlCommand \
  "{throttle: 0.0, steering: 0.0, brake: 1.0}"
```

Then press `Ctrl+C` in these terminals:

1. Throttle publisher
2. Autonomy stack
3. RL training or TensorBoard, if running
4. ROS bridge
5. Foxglove bridge, if running
6. UE5 simulator
7. `websockify`
8. `x11vnc`
9. SSH tunnels on your PC

If the simulator does not exit cleanly:

```bash
pgrep -af 'FSOnline/Binaries/Linux/Blocks|Blocks.sh'
kill <PID>
```

Use `kill -9` only if normal `kill` does not stop the process.

## Troubleshooting

### Browser Connects But Screen Is Black

Wake the display:

```bash
DISPLAY=:1 xset dpms force on
```

Check for the GNOME lock/guard window:

```bash
DISPLAY=:1 xwininfo -root -tree | grep -E 'mutter guard|Formula|Blocks|FSOnline'
```

If the lock screen is present, unlock it in noVNC.

### `x11vnc -auth guess` Fails For `:1`

Check the active display:

```bash
ls -la /tmp/.X11-unix
loginctl list-sessions
```

If there is no `X1` and only `X0` exists, the GX10 is likely at the GDM login screen. Use Path B in step 1.

### noVNC Page Loads But Does Not Connect

Check ports on the GX10:

```bash
ss -ltnp | grep -E ':5900|:6080'
```

Expected:

```text
127.0.0.1:5900  x11vnc
127.0.0.1:6080  websockify
```

If port `6080` is listening but port `5900` is not, restart `x11vnc`.

If `x11vnc` prints `ListenOnTCPPort: Address already in use` or `Error: could not obtain listening port`, port `5900` is already owned by an older VNC backend. Check it:

```bash
ss -ltnp | grep -E ':5900'
pgrep -af 'x11vnc'
```

If the listed process is `x11vnc`, keep it and start/restart only the noVNC proxy:

```bash
websockify --web=/usr/share/novnc/ 127.0.0.1:6080 127.0.0.1:5900
```

If the old backend is stale or stuck, stop both viewer services and restart terminal 1 and terminal 2:

```bash
pgrep -f '[w]ebsockify.*6080' | xargs -r kill
pgrep -f '[x]11vnc.*5900' | xargs -r kill
sleep 2
pgrep -f '[w]ebsockify.*6080' | xargs -r kill -9
pgrep -f '[x]11vnc.*5900' | xargs -r kill -9
```

### Simulator Window Does Not Appear

Confirm the display and Vulkan path:

```bash
DISPLAY=:1 xrandr --query
DISPLAY=:1 vulkaninfo --summary
```

Then start the simulator again without `-nullrhi`.

### ROS Bridge Cannot Connect

Confirm the simulator RPC port:

```bash
nc -vz 127.0.0.1 41451
```

If this fails, the simulator is not ready or did not start the RPC server.

### Car Does Not Move

Check that a publisher is active:

```bash
source /opt/ros/jazzy/setup.bash
source /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2/install/setup.bash

ros2 topic info /fsds/control_command -v
```

`Publisher count` should be greater than `0` while a control publisher is running.

Check speed:

```bash
ros2 topic echo /fsds/gss --once
```

If speed is near zero and no control publisher is active, start the autonomy stack in step 11 or use the manual throttle test in step 13.

## Admin Server Setup Reference

Normal users should not need this section. It records what is already installed/configured on the GX10.

Server packages:

```bash
sudo apt update
sudo apt install -y \
  x11vnc \
  novnc \
  websockify \
  openssh-server \
  netcat-openbsd \
  mesa-utils \
  vulkan-tools
```

VNC password file:

```text
/home/hard/.vnc/passwd
```

The VNC password is configured to match the PC/Linux password for user `hard`. Do not recreate it during normal use. Only recreate it if the file is missing or the team intentionally changes the remote desktop password:

```bash
mkdir -p ~/.vnc
x11vnc -storepasswd ~/.vnc/passwd
```

Expected built artifacts:

```text
/home/hard/Desktop/Formula-Student-Driverless-Simulator/UE5Builds/LinuxArm64/Blocks.sh
/home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2/install/setup.bash
/opt/ros/jazzy/setup.bash
```

The full GX10 UE5/ROS setup history is documented in `SETUP_ROS2_JAZZY_GX10_UE5.md`.
