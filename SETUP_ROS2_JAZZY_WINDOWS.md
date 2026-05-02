# FSDS Setup Notes: Windows Simulator + ROS 2 Jazzy Bridge

Date started: 2026-05-02

This file documents the setup performed in this checkout so the same process can be repeated on another machine.

## Goal

- Run the Formula Student Driverless Simulator on Windows.
- Build and run the ROS 2 bridge with ROS 2 Jazzy in WSL.

## Repository Location

Windows path:

```powershell
D:\Github stuff\Formula-Student-Driverless-Simulator
```

WSL path:

```bash
/mnt/d/Github\ stuff/Formula-Student-Driverless-Simulator
```

The FSDS docs often assume the repo is available at:

```bash
~/Formula-Student-Driverless-Simulator
```

On this machine the repo is on the Windows `D:` drive, so WSL should use a symlink from the Linux home directory to the mounted Windows checkout.

## Initial Project Findings

- Repo contains:
  - `UE4Project/FSOnline.uproject`: Unreal Engine project, EngineAssociation `4.27`
  - `AirSim/`: FSDS hard fork of AirSim shared code
  - `ros2/`: ROS 2 bridge workspace
  - `settings.json`: default simulator and sensor config
- Git LFS is installed on Windows:

```powershell
git-lfs/3.7.0
```

- Submodules were initially not initialized.
- Common Windows build tools were not found in `PATH`:
  - `cl`
  - `msbuild`
- Unreal Engine was not found under:

```powershell
C:\Program Files\Epic Games
```

## Commands Already Run

Initialize submodules:

```powershell
git submodule update --init --recursive
```

Result:

```text
AirSim/external/rpclib -> a663a1598a4b419123b2e13c0ae6a39c91dcf5b8
ros/src/fs_msgs      -> 8a68103031cd7fd45e0062a4fde2e7db318f843a
ros2/src/fs_msgs     -> 4146e5b4889fb92332c9ce5ee42a8081649cbe72
```

Pull Git LFS assets:

```powershell
git lfs pull
```

Create the WSL home symlink expected by FSDS tools:

```bash
ln -sfn /mnt/d/Github\ stuff/Formula-Student-Driverless-Simulator /home/kyriakos/Formula-Student-Driverless-Simulator
```

Verified:

```text
/home/kyriakos/Formula-Student-Driverless-Simulator -> /mnt/d/Github stuff/Formula-Student-Driverless-Simulator
settings.json visible from WSL
```

Add AirSim's Eigen headers manually, because `AirSim/setup.sh` wants sudo for apt packages before it downloads Eigen:

```bash
cd ~/Formula-Student-Driverless-Simulator/AirSim
wget -q -O eigen3.zip https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.zip
rm -rf temp_eigen
unzip -q eigen3.zip -d temp_eigen
mkdir -p AirLib/deps/eigen3
mv temp_eigen/eigen*/Eigen AirLib/deps/eigen3/
rm -rf temp_eigen eigen3.zip
```

Build ROS 2 workspace:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Formula-Student-Driverless-Simulator/ros2
colcon build --symlink-install
```

Result:

```text
Summary: 2 packages finished [2min 33s]
```

Verified installed packages:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Formula-Student-Driverless-Simulator/ros2/install/setup.bash
ros2 pkg executables fsds_ros2_bridge
```

Result:

```text
fsds_ros2_bridge fsds_ros2_bridge
fsds_ros2_bridge fsds_ros2_bridge_camera
```

## WSL Environment Found

WSL distro:

```text
Ubuntu-24.04
WSL version: 2
```

Ubuntu version:

```text
Ubuntu 24.04.4 LTS, noble
```

ROS install:

```text
/opt/ros/jazzy
```

Tools found in WSL:

```bash
colcon
rosdep
python3
cmake 3.28.3
gcc 13.3.0
g++ 13.3.0
make
rsync
unzip
wget
pkg-config
```

Libraries found through `pkg-config`:

```text
yaml-cpp 0.8.0
libcurl 8.5.0
opencv4 4.6.0
```

ROS 2 packages checked and present:

```text
ament_cmake_auto
rclcpp
cv_bridge
image_transport
tf2_ros
sensor_msgs
nav_msgs
geometry_msgs
std_msgs
tf2
```

Current WSL limitations:

- `sudo -n true` fails, so sudo needs an interactive password.
- `rosdep` is installed but not initialized:

```bash
sudo rosdep init
rosdep update
```

These commands need to be run manually inside WSL because they require sudo.

They were not required for this machine because the needed ROS 2 Jazzy packages and native libraries were already present.

## WSL Packages To Install On A New Machine

Install ROS 2 Jazzy from the official ROS 2 Ubuntu deb instructions:

```text
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
```

For Ubuntu 24.04, the official docs state that Jazzy deb packages target Ubuntu Noble 24.04 and install with:

```bash
sudo apt install ros-jazzy-desktop
```

For this FSDS bridge, also install common build and native dependencies:

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  git-lfs \
  wget \
  unzip \
  rsync \
  pkg-config \
  python3-colcon-common-extensions \
  python3-rosdep \
  libyaml-cpp-dev \
  libcurl4-openssl-dev \
  libopencv-dev
```

If you installed `ros-jazzy-ros-base` instead of `ros-jazzy-desktop`, install these ROS packages explicitly:

```bash
sudo apt install -y \
  ros-jazzy-ament-cmake-auto \
  ros-jazzy-rclcpp \
  ros-jazzy-geometry-msgs \
  ros-jazzy-image-transport \
  ros-jazzy-nav-msgs \
  ros-jazzy-sensor-msgs \
  ros-jazzy-std-msgs \
  ros-jazzy-tf2 \
  ros-jazzy-tf2-ros \
  ros-jazzy-cv-bridge
```

## Windows Simulator Requirements

To build/run the Unreal project from source on Windows, install:

1. Unreal Engine 4.27 from Epic Games Launcher.
2. Visual Studio 2019 with:
   - Desktop development with C++
   - Game development with C++
   - Linux development with C++
   - C++ CMake tools for Windows
   - Windows 10 SDK 10.0.18362.0 or compatible
   - .NET Framework 4.7 SDK or compatible
3. Git LFS.

If you only want to run the simulator and do not need to edit Unreal content, the easier path is to use a packaged FSDS release binary for Windows and match the ROS bridge checkout to the same FSDS release tag.

## Windows Runtime Used Here

The current latest GitHub release checked during setup was:

```text
v2.2.0 / 2.2.0
published_at: 2023-01-15T20:28:29Z
release page: https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/tag/v2.2.0
```

Windows asset:

```text
fsds-v2.2.0-windows.zip
size: 601,773,285 bytes
download: https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/download/v2.2.0/fsds-v2.2.0-windows.zip
```

Downloaded and extracted outside the git repo:

```powershell
D:\Github stuff\FSDS-runtime\fsds-v2.2.0-windows.zip
D:\Github stuff\FSDS-runtime\fsds-v2.2.0-windows
```

The extracted simulator contains:

```text
FSDS.exe
settings.json
FSOnline\Binaries\Win64\Blocks.exe
Engine\Extras\Redist\en-us\UE4PrereqSetup_x64.exe
```

Launched on Windows with:

```powershell
$simDir = 'D:\Github stuff\FSDS-runtime\fsds-v2.2.0-windows'
Start-Process -FilePath "$simDir\FSDS.exe" `
  -WorkingDirectory $simDir `
  -ArgumentList '-windowed','-ResX=1280','-ResY=720' `
  -WindowStyle Normal
```

Verified Windows processes:

```text
FSDS.exe running
Blocks.exe running
MainWindowTitle: Formula Student Driverless Simulator Formula Student Driverless Simulator (debug)
```

Verified Windows simulator RPC port:

```powershell
Test-NetConnection -ComputerName 127.0.0.1 -Port 41451
```

Result:

```text
TcpTestSucceeded: True
netstat: 0.0.0.0:41451 LISTENING
```

## Repeatable WSL Setup Steps

Open Ubuntu 24.04 in WSL and run:

```bash
cd ~
ln -sfn "/mnt/d/Github stuff/Formula-Student-Driverless-Simulator" ~/Formula-Student-Driverless-Simulator
cd ~/Formula-Student-Driverless-Simulator
```

Initialize/update source content if not already done from Windows:

```bash
git submodule update --init --recursive
git lfs pull
```

Initialize rosdep if it has not been done on that WSL install:

```bash
sudo rosdep init
rosdep update
```

Install dependencies if `rosdep` reports them missing:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Formula-Student-Driverless-Simulator/ros2
rosdep install --from-paths src --ignore-src -r -y
```

Build the ROS 2 bridge:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Formula-Student-Driverless-Simulator/ros2
colcon build --symlink-install
```

Run the ROS 2 bridge after the Windows simulator is running:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Formula-Student-Driverless-Simulator/ros2/install/setup.bash
ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py host:=<windows-host-ip>
```

For this WSL2 machine, `localhost` from WSL did not connect to the Windows simulator. The WSL default gateway worked:

```bash
ip route | awk '/default/ {print $3; exit}'
```

Result:

```text
192.168.160.1
```

Connection checks:

```bash
nc -vz -w 3 127.0.0.1 41451
nc -vz -w 3 192.168.160.1 41451
```

Results:

```text
127.0.0.1: connection refused
192.168.160.1: connection succeeded
```

Bridge launch command used for the successful connection test:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Formula-Student-Driverless-Simulator/ros2/install/setup.bash
cd ~/Formula-Student-Driverless-Simulator/ros2
ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py host:=192.168.160.1
```

Successful bridge output included:

```text
Connected to the simulator!
AirsimROSWrapper Initialized!
GPS enabled
GSS enabled
IMU enabled
```

The bridge was then started as a background WSL process from PowerShell with logs redirected to:

```powershell
D:\Github stuff\FSDS-runtime\ros2_bridge.out.log
D:\Github stuff\FSDS-runtime\ros2_bridge.err.log
```

Running nodes were verified with:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Formula-Student-Driverless-Simulator/ros2/install/setup.bash
ros2 node list
```

Result:

```text
/fsds/camera/cam1
/fsds/camera/cam2
/fsds/ros_bridge
```

To stop the bridge later:

```bash
pkill -f "ros2 launch fsds_ros2_bridge"
```

To stop the Windows simulator later:

```powershell
Stop-Process -Name FSDS,Blocks
```

## Current Next Steps

- Keep the Windows simulator open while running the ROS 2 bridge.
- Use `host:=192.168.160.1` on this WSL2 machine, or recompute the default gateway with `ip route` after reboot/network changes.
- If the simulator does not launch on another Windows machine, run `Engine\Extras\Redist\en-us\UE4PrereqSetup_x64.exe` from the extracted runtime folder.
- For source-based simulator development, install Unreal Engine 4.27 and Visual Studio 2019 C++ tooling, then build the Unreal project from `UE4Project/FSOnline.uproject`.

## Working Tree Notes

After building from WSL on a Windows-mounted checkout, the `AirSim/external/rpclib` submodule may appear modified from Windows Git due line-ending normalization warnings, even though `git diff` showed no content diff. The relevant files observed were:

```text
AirSim/external/rpclib/doc/pages/versions.md
AirSim/external/rpclib/include/rpc/config.h
AirSim/external/rpclib/include/rpc/version.h
```
