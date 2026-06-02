# GX10 Quick Start Commands

All `GX10` commands assume you are already SSHed into the GX10 or using a terminal on the GX10. Only `Your PC` commands run on your local machine.

GX10 - terminal 0: close old VNC/noVNC services if they are already running.

```bash
pgrep -f '[w]ebsockify.*6080' | xargs -r kill
pgrep -f '[x]11vnc.*5900' | xargs -r kill
sleep 2
pgrep -f '[w]ebsockify.*6080' | xargs -r kill -9
pgrep -f '[x]11vnc.*5900' | xargs -r kill -9
```

GX10 - optional check: confirm ports `5900` and `6080` are free before starting fresh.

```bash
ss -ltnp | grep -E ':(5900|6080)' || true
pgrep -af 'x11vnc|websockify|novnc' || true
```

No output for ports `5900` and `6080` means both viewer services are closed.

GX10 - new terminal 1: start VNC backend.

```bash
x11vnc -display :1 -auth guess -rfbauth ~/.vnc/passwd -localhost -forever -shared -noxdamage -repeat -rfbport 5900
```

GX10 - new terminal 2: start noVNC proxy.

```bash
websockify --web=/usr/share/novnc/ 127.0.0.1:6080 127.0.0.1:5900
```

Your PC - terminal 3: tunnel noVNC.

```bash
ssh -N -L 127.0.0.1:16080:127.0.0.1:6080 hard@100.81.202.64
```

Your PC - browser: open the GX10 desktop.

```text
http://localhost:16080/vnc.html?host=localhost&port=16080
```

If Windows prints `bind [127.0.0.1]:6080: Permission denied`, the local PC port is blocked or reserved. Keep the GX10 noVNC proxy on remote `6080`, but use local `16080` as shown above.

If the browser opens to the Ubuntu login/lock screen and the first login says there was a problem, wait 20-60 seconds and try once more. This usually happens while GNOME is switching from the lock/login greeter into the active `hard` desktop session on display `:1`; once the desktop session finishes waking up, noVNC works normally. Do not start extra `websockify` terminals while waiting. If it keeps happening, rerun terminal 0 to clean old VNC/noVNC processes, then start terminals 1-3 again.

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

GX10 - new terminal 6: start deterministic autonomy, dynamic straight/curve speed, dual-camera cone triangulation, YOLO overlays, Foxglove viz topics, and cone-hit auto reset.

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select fsds_autonomy_msgs fsds_autonomy --symlink-install
CONE_MODEL="$(find /home/hard/.fsds_autonomy/runs/fsds_cones \( -name best.engine -o -name best.onnx -o -name best.pt \) -path '*/weights/*' -printf '%T@ %p\n' 2>/dev/null | sort -nr | awk 'NR==1 {print $2}')"
test -n "$CONE_MODEL" || CONE_MODEL="/home/hard/Desktop/Driverless_FSD_HARD/ros2_ws/src/cone_detection_model/yolo26n.pt"
python3 src/fsds_autonomy/tools/check_yolo_model.py "$CONE_MODEL"
ros2 launch fsds_autonomy fsds_autonomy.launch.py map_dir:=/home/hard/.fsds_autonomy/maps dataset_dir:=/home/hard/.fsds_autonomy/datasets/fsds_cones model_path:="$CONE_MODEL" use_testing_odom:=true auto_reset_enabled:=true dataset_enabled:=false
```

This command does not start live RL. It maps first, drives the middle/centerline with a speed-aware bicycle/pure-pursuit steering preview until the mapper has completed a long loop and sees the orange start/end gate again, progressively raises pre-loop speed as map quality and centerline coverage improve, caps that speed when visible-curve data is insufficient, slows for visible curves, avoids obstacles with a temporary path offset when there is room, stops if the corridor is blocked, saves the map, then uses the saved racing line on later loop-verified runs. The controller publishes `/fsds/control_command` at `100 Hz`, keeps a progress cursor on the chosen drive line so the future steering target advances smoothly instead of being re-picked from scratch every frame as the live map changes, and it lightly filters lateral target movement while using early live-cone fallback. If a live-learning map closes but has not been saved and loaded as `race_from_map`, the behavior planner stays in `LoopVerifyCenterline` and the controller keeps using centerline/edge-recovery safety instead of immediately trusting the generated racing line. If a usable closed-loop saved map already exists, the mapper keeps it as the saved/reference map and builds a separate clean live map on top from LiDAR-plus-stereo-confirmed cones; partial open-loop maps are ignored for race-from-map startup. Race-from-map live observations are not blindly written back, but every later pass can refine the saved map: matched same-color cones within `1.0 m` gently nudge saved cone positions, highly persistent new cones can be added, no saved cones are deleted, and quality/sanity checks plus `/home/hard/.fsds_autonomy/maps/history` backups protect the previous map. It also carries saved and live blue/yellow boundary lines ordered by centerline progress, so losing a nearby cone line briefly does not erase the remembered track shape or connect far-apart cones across the map. Once a closed loop is confirmed and both boundary loops pass endpoint-gap/internal-step checks, Foxglove shades the inner island and an outer boundary band red and the reset monitor treats only the corridor between blue and yellow boundaries, with a `0.60 m` last-resort center safety tripwire from each side, as drivable. The controller also applies a boundary guard that nudges the steering target back toward the middle, slows the car, and brakes if the car drifts too close to either side. Racing lines account for the `1.00 m` vehicle collision width, `0.23 m` cone width, and a `1.35 m` safety/tracking margin; they are generated as smooth outside-apex-outside lines that use the available track width, then clamped inside the blue/yellow cone corridor before publishing/saving. Cone-hit/stuck/forbidden/offtrack reset locations are logged to `/home/hard/.fsds_autonomy/maps/*.reset_events.json`, enriched with nearest line/cone context, published as X markers on `/autonomy/viz/reset_markers`, and fed back into future line generation: repeated reset clusters nudge the live/saved line away from the side where the car got stuck or hit a cone. Runtime map/controller/visible-corridor calculations use cones with confidence `>=0.50`; lower-confidence detections can still appear in debug overlays. Persistent map creation requires LiDAR plus `CameraStereo(cam1+cam2)` agreement by default, requires at least three observations over at least `0.20 s` before publishing/saving a landmark, prunes tentative landmarks that disappear for about `1.0 s`, ignores far/off-corridor cone points, and merges same-color landmarks within about `1.5 m`; raw LiDAR-only clusters and unmatched stereo points are visible for diagnostics but do not become permanent blue/yellow map cones. Overlapping camera boxes that land at nearly the same cone position are de-duplicated before stereo/fusion/mapping, keeping the higher-confidence cone. If the live racing line is sparse, low quality, or too close to live cones, it remains visible for debugging but the car drives the saved/reference line instead. Automatic reset is armed from the simulator cone-hit counter, a sustained zero-speed stuck watchdog, a hard extreme-offtrack watchdog, and the loop-confirmed red forbidden area; leaving the saved racing line does not reset the car by default. Start live RL only after `/autonomy/racing_line` is closed and usable; it waits automatically until the map is ready.

For GPS-assisted mapping from a fresh start/reset, use the same launch with `use_testing_odom:=false`. The estimator then builds `/autonomy/pose` from GPS + GSS + IMU, publishes `/autonomy/gps_pose` for inspection, and uses GPS to correct dead-reckoning drift. Keep `use_testing_odom:=true` when validating against maps already built in the simulator testing-odom frame.

GX10 - optional instead of terminal 6: manual driving with autonomy camera preview only.

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select fsds_autonomy_msgs fsds_autonomy --symlink-install
CONE_MODEL="$(find /home/hard/.fsds_autonomy/runs/fsds_cones \( -name best.engine -o -name best.onnx -o -name best.pt \) -path '*/weights/*' -printf '%T@ %p\n' 2>/dev/null | sort -nr | awk 'NR==1 {print $2}')"
test -n "$CONE_MODEL" || CONE_MODEL="/home/hard/Desktop/Driverless_FSD_HARD/ros2_ws/src/cone_detection_model/yolo26n.pt"
python3 src/fsds_autonomy/tools/check_yolo_model.py "$CONE_MODEL"
ros2 launch fsds_autonomy fsds_autonomy.launch.py map_dir:=/home/hard/.fsds_autonomy/maps dataset_dir:=/home/hard/.fsds_autonomy/datasets/fsds_cones model_path:="$CONE_MODEL" use_testing_odom:=true control_enabled:=false auto_reset_enabled:=false dataset_enabled:=false
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

Useful Foxglove 3D map topics:

```text
/autonomy/viz/map_markers
/autonomy/viz/saved_map_markers
/autonomy/viz/live_map_markers
/autonomy/viz/reset_markers
/autonomy/viz/reference_blue_boundary_path
/autonomy/viz/reference_yellow_boundary_path
/autonomy/viz/live_blue_boundary_path
/autonomy/viz/live_yellow_boundary_path
/autonomy/viz/reference_racing_line_path
/autonomy/viz/live_racing_line_path
/autonomy/viz/current_drive_line_path
```

Use `/autonomy/viz/saved_map_markers` to see the remembered/reference cone positions and saved lines. Use `/autonomy/viz/live_map_markers` to see the live/current map cone positions plus the recent fused cones the car currently sees. Keep `/autonomy/viz/map_markers` enabled for the combined driving view with vehicle pose, obstacles, behavior text, and current drive line. Set the 3D panel fixed frame to `fsds/map`; `/tf` connects `fsds/map` to `fsds/FSCar` so the map and simulator sensor frames share one tree.

Useful Foxglove camera image topics:

```text
/autonomy/viz/camera/cam1_debug
/autonomy/viz/camera/cam2_debug
/autonomy/camera_debug
/autonomy/viz/camera/cam1_run
/autonomy/viz/camera/cam2_run
/autonomy/camera_run
```

Use the debug topics to see every camera detection, confidence label, stereo marker, and projected line. Use the run topics to see only the stereo detections and map/drive lines the car is actually using. The older `/autonomy/viz/camera/cam1_overlay` and `/autonomy/viz/camera/cam2_overlay` topics are kept as debug aliases.

GX10 - optional new terminal: monitor cone-hit auto reset.

```bash
source /opt/ros/jazzy/setup.bash
source /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2/install/setup.bash
ros2 topic echo /autonomy/offtrack_reset_status std_msgs/msg/String
```

GX10 - optional new terminal: monitor mapper persistence, tentative cones, confirmed cones, live map quality, and saved-map refinement version.

```bash
source /opt/ros/jazzy/setup.bash
source /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2/install/setup.bash
ros2 topic echo /autonomy/mapper_diagnostics std_msgs/msg/String
```

GX10 - optional new terminal: monitor LiDAR vs stereo camera cone error.

```bash
source /opt/ros/jazzy/setup.bash
source /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2/install/setup.bash
ros2 topic echo /autonomy/sensor_fusion_diagnostics std_msgs/msg/String
```

GX10 - optional: inspect the latest drive recorder log.

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/fsds_autonomy/tools/analyze_drive_log.py /home/hard/.fsds_autonomy/drive_logs/latest.jsonl --last 600
```

The drive recorder starts with autonomy by default and logs every `/fsds/control_command` with timing, speed, target speed, pose, target source, boundary clearance, cone summary, mapper diagnostics, sensor-fusion diagnostics, and reset status. Logs are written under `/home/hard/.fsds_autonomy/drive_logs`; `latest.jsonl` points at the newest run.

GX10 - optional: replay a recorded control stream. Use `--dry-run` first to verify the file and timing without moving the car.

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/fsds_autonomy/tools/replay_drive_log.py /home/hard/.fsds_autonomy/drive_logs/latest.jsonl --dry-run
```

GX10 - optional new terminal: inspect live/reference blue and yellow boundary memory.

```bash
source /opt/ros/jazzy/setup.bash
source /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2/install/setup.bash
ros2 topic echo /autonomy/viz/live_blue_boundary_path nav_msgs/msg/Path --once
ros2 topic echo /autonomy/viz/live_yellow_boundary_path nav_msgs/msg/Path --once
ros2 topic echo /autonomy/viz/reference_blue_boundary_path nav_msgs/msg/Path --once
ros2 topic echo /autonomy/viz/reference_yellow_boundary_path nav_msgs/msg/Path --once
```

GX10 - optional: check whether the configured YOLO weights are actually cone-trained.

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
CONE_MODEL="$(find /home/hard/.fsds_autonomy/runs/fsds_cones \( -name best.engine -o -name best.onnx -o -name best.pt \) -path '*/weights/*' -printf '%T@ %p\n' 2>/dev/null | sort -nr | awk 'NR==1 {print $2}')"
echo "${CONE_MODEL:-No fine-tuned cone model found}"
test -n "$CONE_MODEL" && python3 src/fsds_autonomy/tools/check_yolo_model.py "$CONE_MODEL"
```

The autonomy launch now prefers the newest `/home/hard/.fsds_autonomy/runs/fsds_cones/*/weights/best.engine`, `best.onnx`, or `best.pt` model automatically. YOLO runs inside the camera detector on `/fsds/cam1/image_color` and `/fsds/cam2/image_color`; LiDAR is detected separately, then `/autonomy/fused_cones` combines LiDAR cones with camera color/confidence/stereo agreement. If no fine-tuned model exists, it falls back to the RC repo `yolo26n.pt`; that file is usually a generic COCO model, so cone boxes will come from the low-trust HSV fallback until a cone model is trained.

GX10 - optional: export the cone YOLO camera model to ONNX or TensorRT. TensorRT `.engine` files are fastest but should be exported on the same GX10/GPU that will run them.

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
CONE_PT="$(find /home/hard/.fsds_autonomy/runs/fsds_cones -path '*/weights/best.pt' -printf '%T@ %p\n' 2>/dev/null | sort -nr | awk 'NR==1 {print $2}')"
test -n "$CONE_PT" || { echo "No trained best.pt found to export"; exit 1; }
python3 src/fsds_autonomy/tools/export_yolo.py --model "$CONE_PT" --format onnx --imgsz 960 --device 0 --half --simplify
python3 src/fsds_autonomy/tools/export_yolo.py --model "$CONE_PT" --format engine --imgsz 960 --device 0 --half --workspace 16
```

After export, rerun the normal autonomy launch. The `CONE_MODEL=...` line above will pick the newest `best.engine`/`best.onnx`/`best.pt`, and the camera detector logs whether it loaded `YOLO TensorRT`, `YOLO ONNX`, or `YOLO PyTorch`.

GX10 - optional new terminal: fine-tune the cone model further for small/far cones. Run this when the simulator is not being actively tested because it uses the GPU heavily.

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
mkdir -p /home/hard/.fsds_autonomy/logs
CONE_MODEL="$(find /home/hard/.fsds_autonomy/runs/fsds_cones -path '*/weights/best.pt' -printf '%T@ %p\n' 2>/dev/null | sort -nr | awk 'NR==1 {print $2}')"
test -n "$CONE_MODEL" || { echo "No existing cone best.pt found to continue fine-tuning from"; exit 1; }
nohup python3 src/fsds_autonomy/tools/train_yolo.py \
  --data /home/hard/.fsds_autonomy/datasets/fsds_cones_sim_20260527T020758Z_color_checked/split/fsds_cones_split.yaml \
  --model "$CONE_MODEL" \
  --epochs 60 \
  --imgsz 1280 \
  --batch 8 \
  --device 0 \
  --cache false \
  --project /home/hard/.fsds_autonomy/runs/fsds_cones \
  --name yolo26n_fsds_far_cones_1280 \
  > /home/hard/.fsds_autonomy/logs/train_yolo_far_cones_1280.log 2>&1 &
tail -f /home/hard/.fsds_autonomy/logs/train_yolo_far_cones_1280.log
```

GX10 - optional: manual reset.

```bash
source /opt/ros/jazzy/setup.bash
source /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2/install/setup.bash
ros2 service call /fsds/reset fs_msgs/srv/Reset "{wait_on_last_task: false}"
```

GX10 - optional new terminal: collect deterministic-controller demonstrations for RL warm-starting.

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/fsds_autonomy/tools/collect_bc_demos.py \
  --out-dir /home/hard/.fsds_autonomy/bc_demos \
  --map-dir /home/hard/.fsds_autonomy/maps \
  --sample-hz 20 \
  --max-duration-sec 300 \
  --stage 1 \
  --residual-label-mode path-speed
```

Run this while the deterministic autonomy stack is driving cleanly. The collector records the same observation layout used by live RL, including fused cones, center/racing/yellow/blue line offsets, preview points, local track curvature/width hints, and nearby prior reset/cone-hit danger-zone hints. The `path-speed` residual labels make the warm-start useful in Stage 1/2, where direct steering/throttle/brake are still supervised away by the deterministic controller.

GX10 - optional new terminal: train a SAC behavior-cloning warm-start from collected demonstrations.

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/fsds_autonomy/tools/train_bc_warmstart.py \
  /home/hard/.fsds_autonomy/bc_demos \
  --artifact sac \
  --epochs 35 \
  --device auto \
  --out-dir /home/hard/.fsds_autonomy/bc_warmstarts
```

Use the generated `sac_bc_warmstart.zip` with live RL by adding `--load-model /home/hard/.fsds_autonomy/bc_warmstarts/RUN_ID/sac_bc_warmstart.zip` to the `train_rl_live.py` command below.

GX10 - optional new terminal: run live full-control RL after the map is closed.

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
mkdir -p /home/hard/.fsds_autonomy/logs
python3 src/fsds_autonomy/tools/train_rl_live.py \
  --algo SAC \
  --steps 200000 \
  --auto-steps \
  --min-steps 50000 \
  --step-chunk 10000 \
  --auto-learning-rate \
  --learning-rate 0.0003 \
  --final-learning-rate 0.00005 \
  --device auto \
  --variable-episode-steps \
  --min-episode-steps 3500 \
  --max-episode-steps 8000 \
  --map-dir /home/hard/.fsds_autonomy/maps \
  --danger-zone-enabled \
  --danger-zone-radius-m 7.0 \
  --danger-zone-penalty 0.35 \
  --start-stage 1 \
  --stage1-residual-scale 0.25 \
  --stage2-residual-scale 0.50 \
  --stage2-min-sectors 12 \
  --stage3-min-sectors 22 \
  --stage2-clean-episodes 3 \
  --stage3-clean-episodes 4 \
  --stage4-clean-laps 2 \
  --reset-status-hold-terminal-sec 1.5 \
  --mistake-budget-enabled \
  --mistake-budget-limit 35 \
  --mistake-recovery-per-step 0.35 \
  --offtrack-grace-steps 20 \
  --reset-bad-grace-steps 15 \
  --low-clearance-terminal-m 0.35 \
  --low-clearance-grace-steps 30 \
  --wrong-direction-grace-steps 25 \
  --no-progress-grace-steps 250 \
  --no-progress-min-delta-m 0.01 \
  --time-penalty 0.0 \
  --sector-time-reference-sec 25 \
  --sector-time-reference-reward 0.5 \
  --sector-time-reward-max 10 \
  --speed-reward 0.04 \
  --speed-reward-power 2.0 \
  --speed-reward-max-per-step 0.40 \
  --speed-reward-min-clearance-m 1.00 \
  --speed-reward-full-clearance-m 1.60 \
  --lap-bonus 250 \
  --lap-time-reference-sec 600 \
  --lap-time-reference-reward 50 \
  --lap-time-reward-max 1000 \
  --reward-log \
  --out /home/hard/.fsds_autonomy/runs/rl_live_full_control
```

The live RL trainer waits until `/autonomy/racing_line.closed_loop` is true, map quality is at least `0.60`, and both blue/yellow boundary loops are available. Its observations include fused camera/LiDAR cone detections, local preview points for the estimated centerline and racing line, local preview points for the yellow/blue boundary lines, local `x/y` offsets to the nearest centerline, racing line, yellow line, and blue line, plus local curvature/width hints and nearby prior reset/cone-hit danger-zone hints. It starts as a residual helper, then promotes itself through stages: small line/speed nudges, larger line/speed nudges, partial direct steering/throttle/brake, then full direct controls. Cone hits keep a large terminal penalty and stuck states still terminate after the stuck watchdog, but offtrack/red-zone, wrong-direction, very-low-clearance, and no-forward-progress states now use grace windows/mistake budget before ending the episode. That lets an agent recover from small mistakes while still resetting once it is clearly too deep into bad behavior or wasting time without moving around the loop. Action-size, action-smoothness, and throttle/brake conflict penalties are scaled by the curriculum authority actually applied to the car, so Stage 1/2 do not punish unused direct steering/throttle/brake exploration. There is no per-step time penalty by default, so safely taking longer is not punished. Sector rewards are sequential gates: after crossing one sector, only the next forward sector can pay, so rocking backward/forward over the same line cannot farm points. Each sector pays a base reward plus an inverse-time bonus, so reaching the next gate in half the time roughly doubles the timing bonus. Speed reward is quadratic by default but only pays while moving forward and safely inside the corridor; it fades out below `1.60 m` clearance and reaches zero at `1.00 m`, so simply running flat out into the red zone is not rewarded. Clean lap completion now pays a base `250` points plus inverse lap-time reward: `50` points at `600 s`, `100` at `300 s`, `200` at `150 s`, `400` at `75 s`, capped by `--lap-time-reward-max`. Each episode now samples its time budget between `3500` and `8000` steps, so the agent is not trained against one fixed horizon and occasional careful laps can keep going longer; truncation still happens if it burns through that sampled budget. Stage promotion is deliberately slower and strictly sequential now: Stage 1 can only unlock Stage 2 after 12 sectors across 3 clean episodes, Stage 2 can only unlock Stage 3 after 22 sectors across 4 clean episodes, and Stage 4 can only unlock after 2 clean laps already driven in Stage 3 partial-direct-control mode. Clean laps from Stage 1/2 no longer count toward full direct-control unlock. The learning rate decays automatically from `0.0003` to `0.00005`; with `--auto-steps`, training runs in chunks and can stop after the minimum step count once progress plateaus inside the max step budget. Every RL step is logged to `reward_steps.jsonl` with reward parts, pose, speed, control command, braking, steering, throttle, raw RL action, scaled RL action, path progress, clearances, crossed sector gates, local line offsets, danger-zone strength, mistake budget/counters, sampled episode step limit, and reset status for debugging. The deterministic controller still applies emergency, close-cone, no-go, and boundary safety overrides after RL blending.

To compare algorithms, run separate training directories with the same map, steps, seed, and curriculum. SAC is the default for continuous steering/throttle/brake; TD3 is another off-policy continuous-control baseline; PPO is on-policy and usually needs more simulator samples.

```bash
python3 src/fsds_autonomy/tools/train_rl_live.py --algo SAC --steps 200000 --seed 7 --out /home/hard/.fsds_autonomy/runs/rl_live_compare/sac_seed7
python3 src/fsds_autonomy/tools/train_rl_live.py --algo TD3 --steps 200000 --seed 7 --out /home/hard/.fsds_autonomy/runs/rl_live_compare/td3_seed7
python3 src/fsds_autonomy/tools/train_rl_live.py --algo PPO --steps 200000 --seed 7 --out /home/hard/.fsds_autonomy/runs/rl_live_compare/ppo_seed7
```

Use TensorBoard plus each `monitor.csv` to compare clean progress, sector count, lap completions, resets/cone hits, minimum clearance, average speed, and reward trend. Do not compare only final reward from one run; compare at least two seeds when the simulator time is available. New live RL runs also write `/home/hard/.fsds_autonomy/runs/.../episode_summaries.jsonl`, one row per episode, with best distance reached, terminal reason, lap time, reward breakdown, speed/control aggregates, clearance stats, sector coverage, stage changes, and start/end pose.

GX10 - optional: summarize how far each RL agent got and why episodes ended.

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/fsds_autonomy/tools/summarize_rl_episode_metrics.py --recent 20
```

The summarizer prefers `episode_summaries.jsonl` when present and falls back to `monitor.csv` for older runs. Use `--json` when you want machine-readable output for later tuning scripts.

Physical random starts now have two paths. The safest path is still simulator spawn-position support: after the UE package is rebuilt to honor `Vehicles.FSCar.X/Y/Z`, the multisim launcher can use `--spawn-mode even --spawn-settings-supported` to place workers at different saved-map sectors without runtime teleports. The current packaged UE5 LinuxArm64 binary still ignores those spawn settings.

There is also an experimental runtime random-start mode for RL generalization. It only activates after the selected curriculum stage, picks safe midpoints between the saved blue/yellow boundary loops, brakes until the car is nearly stopped, pauses the sim, teleports with AirSim `simSetVehiclePose`, skips `/fsds/reset` for that episode, then disables random starts after the first RPC/teleport failure. Test this with one learner before leaving it overnight, because older reset-plus-teleport flows have crashed the Chaos car pawn.

Add these flags to a `train_rl_live.py` command to enable experimental random starts from Stage 2 onward:

```bash
--random-start-enabled \
--random-start-min-stage 2 \
--random-start-port 41451 \
--random-start-skip-reset \
--no-random-start-reset-before-teleport \
--random-start-pause-during-teleport \
--random-start-pre-teleport-brake-sec 0.5 \
--random-start-stop-speed-mps 0.20 \
--random-start-stop-timeout-sec 2.0 \
--random-start-disable-after-failures 1
```

GX10 - optional background watcher: once each multisim learner has at least two clean lap completions and a `10000`-step checkpoint, restart that learner from the checkpoint at Stage 2 with safe reset-to-start episodes.

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
tmux new-session -d -s rl_stage2_watchdog \
  "python3 -u src/fsds_autonomy/tools/stage2_random_start_watchdog.py --poll-sec 60 > /home/hard/.fsds_autonomy/logs/stage2_watchdog.log 2>&1"
tail -f /home/hard/.fsds_autonomy/logs/stage2_watchdog.log
```

The restarted Stage 2 runs write under `/home/hard/.fsds_autonomy/runs/multisim_stage2_random`, load the matching SAC replay buffer automatically when it exists, and keep `--no-random-start-enabled` by default. Add `--enable-random-start` to the watchdog command only after a one-agent smoke test proves runtime random starts are stable on this simulator build.

GX10 - optional faster live RL stack once `/home/hard/.fsds_autonomy/maps/A.json` is a good closed map.

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch fsds_autonomy fsds_autonomy.launch.py \
  map_dir:=/home/hard/.fsds_autonomy/maps \
  use_testing_odom:=true \
  auto_reset_enabled:=true \
  dataset_enabled:=false \
  camera_enabled:=false \
  mapper_enabled:=false \
  raceline_planner_enabled:=false \
  saved_map_publisher_enabled:=true \
  foxglove_visualizer_enabled:=false \
  drive_log_enabled:=false
```

This fast stack skips YOLO/camera overlays, live map refinement, raceline recomputation, Foxglove visualization, dataset writing, and drive logging. It keeps mission state, testing odom, LiDAR cones, sensor fusion, saved closed-map publishing, behavior planning, controller safety, and reset monitoring. Use it only for non-vision throughput tests after the map is already closed. For normal live RL, keep `camera_enabled:=true` so `/autonomy/fused_cones` includes camera cone detections.

For a faster simulator clock, make a copy of `settings.json` with `"ClockSpeed": 1.5` or `"ClockSpeed": 2.0`. Only set `Vehicles.FSCar.Cameras` to `{}` for non-vision throughput tests; camera-enabled live RL should keep `cam1` and `cam2`. Start the simulator with `-settings/path/to/settings_fast.json` with no space and no equals sign, and start the bridge with `settings_path:=/path/to/settings_fast.json`. Keep the RL action interval close to the simulated control interval, for example `--step-dt 0.05` at `ClockSpeed=2.0`. Higher values may destabilize vehicle physics or make the bridge skip too much motion between observations, so test `1.5x` before `2x`.

For multiple simulator instances, each instance needs its own settings file with a unique `ApiServerPort`, plus its own ROS domain so absolute `/fsds/*` and `/autonomy/*` topics do not collide. The ROS 2 bridge supports `api_port:=PORT`.

GX10 - recommended multisim RL launcher. This starts one simulator, one bridge, one autonomy stack, and one SAC learner per agent in `tmux`. Two workers are the current safe default on the GX10 because two camera-enabled UE instances already keep the GPU near full utilization.

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/fsds_autonomy/tools/start_multisim_rl_agents.py \
  --count 2 \
  --spawn-mode start \
  --start-stage 1 \
  --load-rl-model /home/hard/.fsds_autonomy/runs/rl_live_curriculum_patched_20260602T142119Z/checkpoints/sac_live_full_control_30000_steps.zip \
  --run-base /home/hard/.fsds_autonomy/runs/multisim_auto_curriculum
```

For an experimental random-start multisim test, start with one worker first:

```bash
python3 src/fsds_autonomy/tools/start_multisim_rl_agents.py \
  --count 1 \
  --spawn-mode start \
  --rl-random-start-enabled \
  --rl-random-start-min-stage 2 \
  --run-base /home/hard/.fsds_autonomy/runs/multisim_runtime_random_smoke
```

Check them with `tmux ls | grep msim`, `tail -f /home/hard/.fsds_autonomy/logs/msim1_rl.log`, and `tail -f /home/hard/.fsds_autonomy/logs/msim2_rl.log`.

```bash
DISPLAY=:1 ./Blocks.sh -vulkan -windowed -ResX=640 -ResY=360 -nosound -NoVSync -stdout -FullStdOutLogOutput -log -settings/path/to/settings_41451.json -ExecCmds="r.VSync 0,t.MaxFPS 0"
DISPLAY=:1 ./Blocks.sh -vulkan -windowed -ResX=640 -ResY=360 -nosound -NoVSync -stdout -FullStdOutLogOutput -log -settings/path/to/settings_41452.json -ExecCmds="r.VSync 0,t.MaxFPS 0"
ROS_DOMAIN_ID=21 ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py host:=127.0.0.1 api_port:=41451 settings_path:=/path/to/settings_41451.json manual_mode:=false timeout:=120.0
ROS_DOMAIN_ID=22 ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py host:=127.0.0.1 api_port:=41452 settings_path:=/path/to/settings_41452.json manual_mode:=false timeout:=120.0
```

Run one autonomy stack and one `train_rl_live.py` process in the same `ROS_DOMAIN_ID` as each bridge. If starting from `tmux`, export the domain inside the tmux command, for example `bash -lc 'export ROS_DOMAIN_ID=21; ...'`; setting `ROS_DOMAIN_ID=21 tmux new-session ...` may not propagate into the child ROS processes. Separate learners can train in parallel immediately; one shared SAC learner across several live simulators would need a vectorized environment wrapper that starts one ROS process per domain.

GX10 - optional new terminal: run offline residual RL from a saved map for pretraining experiments.

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/fsds_autonomy/tools/train_rl_residual.py --map-dir /home/hard/.fsds_autonomy/maps --track-id A --algo SAC --steps 200000 --out /home/hard/.fsds_autonomy/runs/rl_residual
```

GX10 - optional new terminal: TensorBoard for RL.

```bash
tensorboard --logdir /home/hard/.fsds_autonomy/runs --host 0.0.0.0 --port 6006
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
