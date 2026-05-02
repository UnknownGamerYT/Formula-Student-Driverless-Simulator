# Run Stuff: FSDS Windows Simulator + ROS 2 Jazzy Bridge

This is the normal day-to-day run flow after setup is complete.

## 1. Start The Windows Simulator

Open PowerShell and run this whole block:

```powershell
# Stop any old hidden simulator processes first.
Stop-Process -Name FSDS,Blocks -Force -ErrorAction SilentlyContinue

# Start one clean simulator instance.
$simDir = 'D:\Github stuff\FSDS-runtime\fsds-v2.2.0-windows'
Start-Process "$simDir\FSDS.exe" `
  -WorkingDirectory $simDir `
  -ArgumentList '-windowed','-ResX=1280','-ResY=720'

# Wait until the AirSim RPC server is ready.
do {
  Start-Sleep -Seconds 5
  $rpc = Test-NetConnection -ComputerName 127.0.0.1 -Port 41451 -WarningAction SilentlyContinue
  $rpc.TcpTestSucceeded
} until ($rpc.TcpTestSucceeded)
```

Optional check:

```powershell
Get-Process FSDS,Blocks -ErrorAction SilentlyContinue
Test-NetConnection -ComputerName 127.0.0.1 -Port 41451
```

`TcpTestSucceeded` should be `True`.

Wait for this before starting the ROS bridge. The Unreal window can appear before the AirSim RPC server is ready.

If Unreal shows `Reflection captures need to be rebuilt`, ignore it for normal simulator use. It is a visual/editor warning and does not stop ROS or AirSim RPC.

## 2. Start The ROS 2 Bridge In WSL

Open Ubuntu/WSL:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Formula-Student-Driverless-Simulator/ros2/install/setup.bash
cd ~/Formula-Student-Driverless-Simulator/ros2

HOST_IP=$(ip route | awk '/default/ {print $3; exit}')
```

For keyboard control in the simulator while ROS is connected, start the bridge in manual mode:

```bash
ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py host:=$HOST_IP manual_mode:=true timeout:=60.0
```

For API-only control from ROS, start the bridge without manual mode:

```bash
ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py host:=$HOST_IP timeout:=60.0
```

Leave this terminal open.

Successful output includes:

```text
Connected to the simulator!
AirsimROSWrapper Initialized!
```

Without `manual_mode:=true`, the ROS bridge takes API control of the car.

## 3. Verify ROS Data

Open a second Ubuntu/WSL terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Formula-Student-Driverless-Simulator/ros2/install/setup.bash

ros2 node list
ros2 topic list | grep fsds
ros2 topic echo /fsds/gps --once
```

Expected nodes:

```text
/fsds/camera/cam1
/fsds/camera/cam2
/fsds/ros_bridge
```

## 4. Visualize In Foxglove

Foxglove is enough for normal topic inspection, camera viewing, plots, and lidar/odom debugging. RViz is optional and mostly useful for deeper TF/frame debugging.

With the simulator and ROS bridge already running, open another Ubuntu/WSL terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Formula-Student-Driverless-Simulator/ros2/install/setup.bash

ros2 run foxglove_bridge foxglove_bridge
```

Expected output:

```text
Server listening on port 8765
```

In Foxglove, connect to:

```text
ws://localhost:8765
```

If `localhost` does not work from Windows, get the WSL IP:

```bash
hostname -I
```

Then connect Foxglove to:

```text
ws://<WSL_IP>:8765
```

List the exact available topics:

```bash
ros2 topic list | grep -E 'fsds|tf'
```

Useful topics to add in Foxglove:

```text
/fsds/gps
/fsds/imu
/fsds/gss
/fsds/testing_only/odom
/fsds/testing_only/track
/fsds/lidar/Lidar1
/fsds/lidar/Lidar2
/tf_static
```

Camera topic names can differ by bridge version. Find them with:

```bash
ros2 topic list | grep camera
```

Common camera topics:

```text
/fsds/camera/cam1/image_color
/fsds/camera/cam1/camera_info
/fsds/camera/cam2/image_color
/fsds/camera/cam2/camera_info
```

Suggested Foxglove panels:

- 3D panel: odometry, lidar, and `/tf_static`
- Image panel: camera image topic
- Plot panel: `/fsds/gss` or `/fsds/imu`
- Raw Messages panel: `/fsds/gps` and `/fsds/testing_only/extra_info`

## 5. Stop Everything

Stop the bridge:

```bash
Ctrl+C
```

Or from another WSL terminal:

```bash
pkill -f "ros2 launch fsds_ros2_bridge"
```

Stop the simulator from PowerShell:

```powershell
Stop-Process -Name FSDS,Blocks
```

## Troubleshooting

If WSL cannot connect to the simulator, check the host IP and port:

```bash
HOST_IP=$(ip route | awk '/default/ {print $3; exit}')
echo $HOST_IP
nc -vz -w 3 $HOST_IP 41451
```

If this fails but PowerShell says port `41451` is open, Windows Firewall may be blocking WSL. Allow the simulator on private networks if Windows asks.

If `localhost` fails from WSL, that is expected on this setup. Use the `HOST_IP` command above.

If the bridge times out on `getServerVersion`, check for duplicate simulator processes:

```powershell
Get-Process FSDS,Blocks -ErrorAction SilentlyContinue
netstat -ano | Select-String ':41451'
```

If more than one `Blocks` or `FSDS` process is running, restart cleanly:

```powershell
Stop-Process -Name FSDS,Blocks -Force -ErrorAction SilentlyContinue
$simDir = 'D:\Github stuff\FSDS-runtime\fsds-v2.2.0-windows'
Start-Process "$simDir\FSDS.exe" -WorkingDirectory $simDir -ArgumentList '-windowed','-ResX=1280','-ResY=720'
```

Then wait until:

```powershell
Test-NetConnection -ComputerName 127.0.0.1 -Port 41451
```

returns `TcpTestSucceeded: True`.

If the simulator machine is slow and the bridge still starts too early, launch with a larger timeout:

```bash
ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py host:=$HOST_IP manual_mode:=true timeout:=120.0
```
