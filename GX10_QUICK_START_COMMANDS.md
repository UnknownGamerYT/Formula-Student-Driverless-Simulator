# GX10 Quick Start Commands

All `GX10` commands assume you are already SSHed into the GX10 or using a terminal on the GX10. Only `Your PC` commands run on your local machine.

GX10 - optional check: see whether VNC/noVNC are already running.

```bash
ss -ltnp | rg ':(5900|6080)\b' || true
pgrep -af 'x11vnc|websockify|novnc' || true
```

If port `5900` already shows an `x11vnc` process, skip terminal 1 and continue with terminal 2. `Address already in use` means the VNC backend is already running, not that the display failed.

GX10 - new terminal 1: start VNC backend if port `5900` is not already in use.

```bash
x11vnc -display :1 -auth guess -rfbauth ~/.vnc/passwd -localhost -forever -shared -rfbport 5900
```

GX10 - optional: restart stale VNC/noVNC processes if the browser cannot connect or the old process is stuck.

```bash
pkill -f 'x11vnc .*5900' || true
pkill -f 'websockify .*6080' || true
```

GX10 - new terminal 2: start noVNC proxy.

```bash
websockify --web=/usr/share/novnc/ 127.0.0.1:6080 127.0.0.1:5900
```

Your PC - terminal 3: tunnel noVNC.

```bash
ssh -N -L 6080:localhost:6080 hard@100.81.202.64
```

Your PC - browser: open the GX10 desktop.

```text
http://localhost:6080/vnc.html?host=localhost&port=6080
```

GX10 - new terminal 4: start the UE5 simulator.

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/UE5Builds/LinuxArm64
DISPLAY=:1 ./Blocks.sh -vulkan -windowed -ResX=1280 -ResY=720 -nosound -NoVSync -stdout -FullStdOutLogOutput -log -ExecCmds="r.VSync 0,t.MaxFPS 0,stat fps,stat unit,stat gpu"
```

GX10 - new terminal 5: start the FSDS ROS 2 bridge for autonomous/API driving.

```bash
source /opt/ros/jazzy/setup.bash
source /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2/install/setup.bash
ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py host:=127.0.0.1 manual_mode:=false timeout:=120.0
```

Use this bridge mode for terminal 6 self-driving. It enables ROS API control, so simulator keyboard vehicle input is disabled while this bridge is running.

GX10 - optional instead of terminal 5: start the bridge for manual keyboard driving.

```bash
source /opt/ros/jazzy/setup.bash
source /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2/install/setup.bash
ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py host:=127.0.0.1 manual_mode:=true timeout:=120.0
```

Use only one bridge mode at a time. Manual keyboard driving needs `manual_mode:=true`; otherwise the ROS bridge keeps API control of the car.

GX10 - new terminal 6: start deterministic autonomy, dynamic straight/curve speed, dual-camera cone triangulation, YOLO overlays, Foxglove viz topics, and auto reset.

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select fsds_autonomy_msgs fsds_autonomy --symlink-install
ros2 launch fsds_autonomy fsds_autonomy.launch.py map_dir:=/home/hard/.fsds_autonomy/maps dataset_dir:=/home/hard/.fsds_autonomy/datasets/fsds_cones auto_reset_enabled:=true dataset_enabled:=false
```

This command does not start live RL. It maps first, speeds up only when the visible cone corridor looks straight, slows for visible curves, avoids obstacles with a temporary path offset when there is room, stops if the corridor is blocked, saves the map, then uses the saved racing line on later runs. Start residual RL only after a map exists.

GX10 - optional instead of terminal 6: manual driving with autonomy camera preview only.

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select fsds_autonomy_msgs fsds_autonomy --symlink-install
ros2 launch fsds_autonomy fsds_autonomy.launch.py map_dir:=/home/hard/.fsds_autonomy/maps dataset_dir:=/home/hard/.fsds_autonomy/datasets/fsds_cones control_enabled:=false auto_reset_enabled:=false dataset_enabled:=false
```

This preview command keeps camera detection, stereo triangulation, confidence overlays, map/racing-line projection, behavior preview, and Foxglove topics running while you manually drive. It does not publish `/fsds/control_command` and it does not auto-reset the simulator.

For this manual preview mode, terminal 5 must be the bridge command with `manual_mode:=true`. Click inside the noVNC/Unreal window before pressing keys. Vehicle controls are arrow keys, Space for foot brake, and End for handbrake. Camera controls are `M` for manual camera, arrow keys to move the camera, PageUp/PageDown for vertical movement, `W/S` pitch, `A/D` yaw, `Q/E` roll, `/` chase view, and `F` FPV.

GX10 - new terminal 7: start Foxglove bridge.

```bash
source /opt/ros/jazzy/setup.bash
source /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2/install/setup.bash
ros2 run foxglove_bridge foxglove_bridge
```

Your PC - terminal 8: tunnel Foxglove.

```bash
ssh -N -L 8765:localhost:8765 hard@100.81.202.64
```

Your PC - Foxglove Studio: connect.

```text
ws://localhost:8765
```

GX10 - optional new terminal: monitor auto reset.

```bash
source /opt/ros/jazzy/setup.bash
source /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2/install/setup.bash
ros2 topic echo /autonomy/offtrack_reset_status std_msgs/msg/String
```

GX10 - optional new terminal: monitor LiDAR vs stereo camera cone error.

```bash
source /opt/ros/jazzy/setup.bash
source /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2/install/setup.bash
ros2 topic echo /autonomy/sensor_fusion_diagnostics std_msgs/msg/String
```

GX10 - optional: check whether the configured YOLO weights are actually cone-trained.

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/fsds_autonomy/tools/check_yolo_model.py /home/hard/Desktop/Driverless_FSD_HARD/ros2_ws/src/cone_detection_model/yolo26n.pt
```

If this prints `NOT_A_CONE_MODEL`, camera boxes are coming from the low-trust HSV fallback until a fine-tuned cone model is trained and launched with `model_path:=...`.

GX10 - optional: manual reset.

```bash
source /opt/ros/jazzy/setup.bash
source /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2/install/setup.bash
ros2 service call /fsds/reset fs_msgs/srv/Reset "{wait_on_last_task: false}"
```

GX10 - optional new terminal: run residual RL after a map exists.

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/fsds_autonomy/tools/train_rl_residual.py --map-dir /home/hard/.fsds_autonomy/maps --track-id A --algo SAC --steps 200000 --out /home/hard/.fsds_autonomy/runs/rl_residual
```

GX10 - optional new terminal: TensorBoard for RL.

```bash
tensorboard --logdir /home/hard/.fsds_autonomy/runs/rl_residual --host 0.0.0.0 --port 6006
```

Your PC - optional: tunnel TensorBoard.

```bash
ssh -N -L 6006:localhost:6006 hard@100.81.202.64
```

Your PC - browser: open TensorBoard.

```text
http://localhost:6006
```

GX10 - stop car.

```bash
source /opt/ros/jazzy/setup.bash
source /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2/install/setup.bash
ros2 topic pub --once /fsds/control_command fs_msgs/msg/ControlCommand "{throttle: 0.0, steering: 0.0, brake: 1.0}"
```
