# FSDS Setup Notes: ASUS Ascent GX10 + ROS 2 Jazzy + UE5 Migration

Date started: 2026-05-18

This file documents the setup performed on the ASUS Ascent GX10 so the process can be repeated and resumed. The GX10 is the native ROS 2/autonomy machine. The current UE4 FSDS runtime is not expected to run natively on this machine because this machine is ARM64 and the released FSDS simulator binaries are x86_64/Win64.

## Goal

- Set up ROS 2 Jazzy and the FSDS ROS 2 bridge on the GX10.
- Keep the GX10 ready to connect to a simulator running on another machine over TCP port `41451`.
- Record every command attempted, the result, and the current blockers.
- Record the UE4-to-UE5 migration areas that must be updated before the simulator can be tested on GX10-compatible hardware.

## Machine State

Command:

```bash
date -u '+%Y-%m-%d %H:%M:%S UTC'
uname -a
dpkg --print-architecture
cat /etc/os-release
```

Result:

```text
2026-05-18 13:56:53 UTC
Linux gx10-d190 6.17.0-1014-nvidia #14-Ubuntu SMP PREEMPT_DYNAMIC Tue Mar 17 19:01:40 UTC 2026 aarch64 aarch64 aarch64 GNU/Linux
arm64
Ubuntu 24.04.4 LTS / noble
```

GPU check:

```bash
nvidia-smi
```

Result:

```text
NVIDIA-SMI 580.142
Driver Version: 580.142
CUDA Version: 13.0
GPU: NVIDIA GB10
```

Repository check:

```bash
git status --short
git rev-parse --abbrev-ref HEAD
git rev-parse HEAD
git submodule status --recursive
```

Result:

```text
Branch: master
HEAD: 9fa16de49c6a7d3de3871cd1c21229cfa31e5358
Submodules:
  a663a1598a4b419123b2e13c0ae6a39c91dcf5b8 AirSim/external/rpclib (v2.3.0)
  8a68103031cd7fd45e0062a4fde2e7db318f843a ros/src/fs_msgs (heads/master)
  4146e5b4889fb92332c9ce5ee42a8081649cbe72 ros2/src/fs_msgs (remotes/origin/ros2)
```

Tooling found before setup:

```bash
docker --version
docker compose version
nvidia-ctk --version
```

Result:

```text
Docker version 29.2.1, build a5c7197
Docker Compose version v5.0.2
NVIDIA Container Toolkit CLI version 1.19.0
```

Missing before setup:

```text
ros2
colcon
rosdep
git-lfs
clang / clang-16
vulkaninfo
glxinfo
```

Important permission state:

```bash
sudo -n true
groups
```

Result:

```text
sudo -n true failed: sudo: a password is required
groups: hard adm sudo audio dip plugdev users lpadmin ollama
```

This means apt installs, `rosdep init`, and Docker group setup cannot be completed non-interactively in this session.

## Why Old FSDS Runtime Does Not Run On GX10

The latest upstream FSDS release checked is still `v2.2.0`, published on 2023-01-15. The Linux release was inspected directly.

Command:

```bash
unzip -q /tmp/fsds-v2.2.0-linux.zip 'FSOnline/Binaries/Linux/Blocks' -d /tmp/fsds-linux-check
file /tmp/fsds-linux-check/FSOnline/Binaries/Linux/Blocks
```

Result:

```text
/tmp/fsds-linux-check/FSOnline/Binaries/Linux/Blocks:
ELF 64-bit LSB executable, x86-64, dynamically linked,
interpreter /lib64/ld-linux-x86-64.so.2
```

Conclusion:

```text
The GX10 is arm64/aarch64. The released Linux simulator binary is x86-64.
The old packaged FSDS runtime cannot run natively on this GX10.
```

The Windows release is Win64 and also cannot run natively on this Linux ARM64 host.

## Packages Installed

ROS 2 apt source was staged but could not be installed because sudo requires an interactive password.

Command:

```bash
ROS_APT_SOURCE_VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | awk -F'"' '/tag_name/ {print $4; exit}')
. /etc/os-release
DEB="/tmp/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME:-$VERSION_CODENAME}_all.deb"
URL="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME:-$VERSION_CODENAME}_all.deb"
curl -L --fail -o "$DEB" "$URL"
ls -lh "$DEB"
```

Result:

```text
ROS_APT_SOURCE_VERSION=1.2.0
Downloaded: /tmp/ros2-apt-source_1.2.0.noble_all.deb
Size: 4.4K
```

Command:

```bash
sudo -n dpkg -i /tmp/ros2-apt-source_*.noble_all.deb
```

Result:

```text
FAILED: sudo: a password is required
```

Command:

```bash
sudo -n apt install -y \
  software-properties-common \
  curl \
  ros-jazzy-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  git-lfs \
  clang-16 \
  libyaml-cpp-dev \
  libcurl4-openssl-dev \
  libopencv-dev \
  vulkan-tools \
  mesa-utils \
  ros-jazzy-foxglove-bridge \
  netcat-openbsd
```

Result:

```text
FAILED: sudo: a password is required
```

Interactive sudo install that was run to continue:

```bash
sudo dpkg -i /tmp/ros2-apt-source_1.2.0.noble_all.deb
sudo apt update
sudo apt install -y \
  software-properties-common \
  curl \
  ros-jazzy-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  git-lfs \
  clang-16 \
  libyaml-cpp-dev \
  libcurl4-openssl-dev \
  libopencv-dev \
  vulkan-tools \
  mesa-utils \
  ros-jazzy-foxglove-bridge \
  netcat-openbsd
```

Note:

```text
Do not install a package named clang++-16 on Ubuntu 24.04.
The apt package is clang-16; it provides the Clang compiler tooling, including the clang++-16 binary.
If a paste accidentally creates a package name like netcat-openbsdlove-bridge, rerun the corrected package block above.
The corrected install completed successfully after this package-name fix.
```

Resume after interactive sudo install:

```bash
dpkg-query -W -f='${Package}\t${Version}\t${Architecture}\t${db:Status-Abbrev}\n' \
  ros-jazzy-desktop ros-dev-tools python3-colcon-common-extensions python3-rosdep \
  git-lfs clang-16 libyaml-cpp-dev libcurl4-openssl-dev libopencv-dev \
  vulkan-tools mesa-utils ros-jazzy-foxglove-bridge netcat-openbsd
```

Result:

```text
clang-16                              1:16.0.6-23ubuntu4                 arm64 ii
git-lfs                               3.4.1-1ubuntu0.4+esm1              arm64 ii
libcurl4-openssl-dev                  8.5.0-2ubuntu10.9                  arm64 ii
libopencv-dev                         4.6.0+dfsg-13.1ubuntu1             arm64 ii
libyaml-cpp-dev                       0.8.0+dfsg-6build1                 arm64 ii
mesa-utils                            9.0.0-2                            arm64 ii
netcat-openbsd                        1.226-1ubuntu2                     arm64 ii
python3-colcon-common-extensions      0.3.0-100                          all   ii
python3-rosdep                        0.26.0-1                           all   ii
ros-dev-tools                         1.0.1                              all   ii
ros-jazzy-desktop                     0.11.0-1noble.20260413.045119      arm64 ii
ros-jazzy-foxglove-bridge             3.2.6-1noble.20260412.172743       arm64 ii
vulkan-tools                          1.4.328.1+dfsg1-1~1                arm64 ii
```

ROS 2 is available after sourcing:

```bash
source /opt/ros/jazzy/setup.bash
ros2 --help
ros2 pkg list | rg '^(demo_nodes_cpp|demo_nodes_py|foxglove_bridge|rclcpp|cv_bridge)$'
```

Result:

```text
ros2: PASS
cv_bridge
demo_nodes_cpp
demo_nodes_py
foxglove_bridge
rclcpp
```

## Repository Runtime Setup

FSDS expects the checkout at `~/Formula-Student-Driverless-Simulator`.

Command:

```bash
ln -sfn /home/hard/Desktop/Formula-Student-Driverless-Simulator "$HOME/Formula-Student-Driverless-Simulator"
ls -ld "$HOME/Formula-Student-Driverless-Simulator"
test -f "$HOME/Formula-Student-Driverless-Simulator/settings.json"
```

Result:

```text
/home/hard/Formula-Student-Driverless-Simulator -> /home/hard/Desktop/Formula-Student-Driverless-Simulator
settings.json readable via FSDS home symlink: PASS
```

Git LFS command:

```bash
git lfs install
git lfs pull
```

Result:

```text
FAILED: git: 'lfs' is not a git command
```

Manual step required after `git-lfs` is installed:

```bash
git lfs install
git lfs pull
```

Post-install retry:

```bash
git lfs install
git lfs pull
```

Result:

```text
Updated Git hooks.
Git LFS initialized.
FAILED:
batch response: This repository exceeded its LFS budget.
Failed to fetch some objects from 'https://github.com/UnknownGamerYT/Formula-Student-Driverless-Simulator.git/info/lfs'
```

This is a remote repository/account bandwidth quota blocker, not a GX10 setup problem.

## AirSim / AirLib Preparation

`AirSim/setup.sh` cannot complete fully without sudo because it installs compiler packages. Eigen was installed manually because it does not require root.

Command:

```bash
cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/AirSim
wget -q -O /tmp/fsds-eigen3.zip https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.zip
rm -rf /tmp/fsds-temp-eigen
unzip -q /tmp/fsds-eigen3.zip -d /tmp/fsds-temp-eigen
mkdir -p AirLib/deps/eigen3
mv /tmp/fsds-temp-eigen/eigen*/Eigen AirLib/deps/eigen3/
rm -rf /tmp/fsds-temp-eigen /tmp/fsds-eigen3.zip
ls -ld AirLib/deps/eigen3/Eigen
```

Result:

```text
Eigen installed manually at AirLib/deps/eigen3/Eigen
```

First AirLib build attempt:

```bash
timeout 120 bash AirSim/build.sh
```

Result:

```text
FAILED:
Could not find compiler set in environment variable CC: clang-16.
CMAKE_C_COMPILER not set, after EnableLanguage
CMAKE_CXX_COMPILER not set, after EnableLanguage
```

Compiler status after package install:

```text
clang-16 and clang++-16 are available at /usr/bin/clang-16 and /usr/bin/clang++-16.
The temporary compiler fallback was removed from AirSim/build.sh.
```

Verification command:

```bash
command -v clang-16
command -v clang++-16
clang-16 --version | head -2
clang++-16 --version | head -2
```

Result:

```text
/usr/bin/clang-16
/usr/bin/clang++-16
Ubuntu clang version 16.0.6 (23ubuntu4)
Target: aarch64-unknown-linux-gnu
```

Strict Clang AirLib rebuild:

```bash
timeout 600 bash AirSim/build.sh
```

Result:

```text
PASS
export CC=clang-16
export CXX=clang++-16
Built target rpc
Built target AirLib
AirSim libraries are built and installed.
```

Built artifacts:

```bash
ls -lh AirSim/AirLib/lib/libAirLib.a AirSim/AirLib/deps/rpclib/lib/librpc.a
file AirSim/build_debug/output/lib/libAirLib.a AirSim/build_debug/output/lib/librpc.a
```

Result:

```text
AirSim/AirLib/lib/libAirLib.a: 34M
AirSim/AirLib/deps/rpclib/lib/librpc.a: 13M
AirSim/build_debug/output/lib/libAirLib.a: current ar archive
AirSim/build_debug/output/lib/librpc.a: current ar archive
```

Note:

```text
AirSim/build.sh also rsyncs AirLib into UE4Project/Plugins/AirSim/Source/AirLib.
The generated build output is ignored by git.
AirSim/build.sh is back to the original strict Clang compiler selection and is no longer modified.
```

Post-fallback-removal validation:

```bash
bash -n AirSim/build.sh
rg -n "fallback|falling back|gcc/g\\+\\+|use_compiler_or_fallback" AirSim/build.sh || true
timeout 600 bash AirSim/build.sh
source /opt/ros/jazzy/setup.bash
cd ros2
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Result:

```text
PASS:
AirSim/build.sh contains no compiler fallback code.
AirLib rebuild used CC=clang-16 and CXX=clang++-16.
ROS 2 workspace rebuild passed: 2 packages finished [0.59s].
```

## ROS 2 Build

Initial rosdep check before initialization:

```bash
test -f /etc/ros/rosdep/sources.list.d/20-default.list && echo present || echo missing
rosdep update
```

Result:

```text
rosdep init file: missing
ERROR: no sources directory exists on the system meaning rosdep has not yet been initialized.
```

Attempted non-interactive init:

```bash
sudo -n rosdep init
```

Result:

```text
FAILED: sudo: a password is required
```

Interactive rosdep initialization was then run:

```bash
sudo rosdep init
rosdep update
```

Result:

```text
PASS
Wrote /etc/ros/rosdep/sources.list.d/20-default.list
updated cache in /home/hard/.ros/rosdep/sources.cache
```

Dependency reconciliation:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Formula-Student-Driverless-Simulator/ros2
rosdep install --from-paths src --ignore-src -r -y
```

Result:

```text
PASS
#All required rosdeps installed successfully
```

The bridge build was first attempted after the apt packages were installed, then repeated after rosdep succeeded.

Command:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Formula-Student-Driverless-Simulator/ros2
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Result:

```text
PASS
Summary: 2 packages finished [44.2s]
1 package had stderr output: fsds_ros2_bridge
```

The stderr output was warning-only from upstream AirSim/rpclib/Eigen headers plus an ament header-install warning. No build error occurred.

Clean rebuild after `rosdep install`:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Formula-Student-Driverless-Simulator/ros2
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Result:

```text
PASS
Summary: 2 packages finished [0.67s]
```

Verification:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Formula-Student-Driverless-Simulator/ros2/install/setup.bash
ros2 pkg executables fsds_ros2_bridge
ros2 pkg prefix fs_msgs
ros2 pkg prefix fsds_ros2_bridge
```

Result:

```text
fsds_ros2_bridge fsds_ros2_bridge
fsds_ros2_bridge fsds_ros2_bridge_camera
/home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2/install/fs_msgs
/home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2/install/fsds_ros2_bridge
```

## ROS-Only Smoke Tests

Initial command before sourcing/installing ROS:

```bash
ros2 run demo_nodes_cpp talker
```

Result:

```text
FAILED: ros2: command not found
```

ROS pub/sub smoke test after install:

```bash
source /opt/ros/jazzy/setup.bash
timeout 8 ros2 run demo_nodes_cpp talker > /tmp/fsds_talker.log 2>&1 &
timeout 5 ros2 run demo_nodes_py listener > /tmp/fsds_listener.log 2>&1
```

Result:

```text
PASS
talker published: Hello World: 1..6
listener heard: Hello World: 3..6
```

Foxglove bridge check:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run foxglove_bridge foxglove_bridge
```

Result:

```text
PASS
Server listening on port 8765
```

Bridge launch check without a simulator:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Formula-Student-Driverless-Simulator/ros2/install/setup.bash
ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py host:=127.0.0.1 timeout:=1.0
```

Result:

```text
PASS for launch wiring / expected simulator absence
Started:
  /fsds/camera/cam1
  /fsds/camera/cam2
  /fsds/ros_bridge

Expected failure:
  Failed connecting to RPC server (airsim). Is the simulator running?
```

This confirms the ROS launch file, settings parsing, camera node creation, and bridge executable paths work. It cannot fully connect until a simulator is listening on TCP `41451`.

## Docker / GPU Validation

Docker and NVIDIA Container Toolkit were already installed. The user was added to the `docker` group interactively after non-interactive sudo failed.

Initial command:

```bash
docker info --format 'ServerVersion={{.ServerVersion}} Architecture={{.Architecture}} Runtimes={{json .Runtimes}} DefaultRuntime={{.DefaultRuntime}}'
```

Initial result:

```text
FAILED:
permission denied while trying to connect to the docker API at unix:///var/run/docker.sock
```

Non-interactive group fix attempt:

```bash
sudo -n usermod -aG docker hard
```

Result:

```text
FAILED: sudo: a password is required
```

Post-user-action group check:

```bash
id
groups
getent group docker
```

Result:

```text
uid=1000(hard) gid=1000(hard) groups=1000(hard),4(adm),27(sudo),29(audio),30(dip),46(plugdev),100(users),122(lpadmin),983(ollama)
hard adm sudo audio dip plugdev users lpadmin ollama
docker:x:988:hard
```

Conclusion:

```text
The /etc/group membership is fixed, but this shell has not picked up the docker group yet.
Plain docker commands will require logout/login, reboot, or a new shell with docker as an active group.
Until then, sg docker -c '<command>' works for validation.
```

Docker info validation through the active-group workaround:

```bash
sg docker -c 'docker info --format "ServerVersion={{.ServerVersion}} Architecture={{.Architecture}} DefaultRuntime={{.DefaultRuntime}} Runtimes={{json .Runtimes}}"'
```

Result:

```text
ServerVersion=29.2.1 Architecture=aarch64 DefaultRuntime=runc
Runtimes include runc and io.containerd.runc.v2.
```

NVIDIA container smoke test:

```bash
sg docker -c 'docker run --rm --gpus all nvcr.io/nvidia/cuda:13.0.0-base-ubuntu24.04 nvidia-smi'
```

Result:

```text
PASS:
NVIDIA-SMI 580.142
Driver Version: 580.142
CUDA Version: 13.0
GPU 0: NVIDIA GB10
```

After logout/login or reboot, the expected direct commands are:

```bash
docker info
docker run --rm --gpus all nvcr.io/nvidia/cuda:13.0.0-base-ubuntu24.04 nvidia-smi
```

## Remote Simulator Networking

Current GX10 network interfaces:

```bash
ip -brief addr show
ip route show default
```

Result:

```text
lo          127.0.0.1/8
enP7s7      10.7.72.10/24
wlP9s9      10.157.1.68/20
tailscale0  100.81.202.64/32
docker0     172.17.0.1/16

default via 10.157.0.1 dev wlP9s9 src 10.157.1.68 metric 600
default via 10.7.72.1 dev enP7s7 src 10.7.72.10 metric 20100
```

No simulator RPC port is currently listening locally:

```bash
ss -ltnup | rg '(:41451|:8765|:7400|:7410|:11811)'
```

Result:

```text
No matching listeners.
```

Use this check once a UE5.5 Chaos FSDS-compatible simulator is running on another machine:

```bash
nc -vz <SIM_HOST_IP> 41451
```

Use this bridge command once ROS 2 is installed and the simulator RPC check passes:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Formula-Student-Driverless-Simulator/ros2/install/setup.bash
ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py host:=<SIM_HOST_IP> timeout:=120.0
```

Expected bridge output:

```text
Connected to the simulator!
AirsimROSWrapper Initialized!
```

## UE4 To UE5 Migration Notes

The existing simulator project is UE4.27 and PhysX-based. These items must be migrated before a UE5 runtime can be tested.

Verified UE4 engine binding:

```text
UE4Project/FSOnline.uproject:
  EngineAssociation: 4.27
```

Required update:

```text
Change EngineAssociation to the chosen UE5 version after the UE5 project has been opened/converted intentionally.
Required simulator target: UE5.5 with Chaos Vehicles. Cosys-AirSim v3.3 may be used only as a UE5.5 AirSim base; the FSDS vehicle runtime must be Chaos-based and must preserve the existing RPC contract.
```

Verified UE4 plugin dependency:

```text
UE4Project/Plugins/AirSim/AirSim.uplugin:
  Plugin dependency: PhysXVehicles
```

Required update:

```text
Remove `PhysXVehicles`; replace the vehicle runtime with UE5 Chaos Vehicles.
Use a UE5.5-compatible AirSim plugin path and migrate the vehicle runtime to Chaos Vehicles. Do not retain `PhysXVehicles` in the migrated simulator.
```

Verified UE4 module dependencies:

```text
UE4Project/Plugins/AirSim/Source/AirSim.Build.cs:
  PhysXVehicles
  PhysXVehicleLib
  PhysX
  APEX
```

Required update:

```text
Replace PhysX/APEX module dependencies with UE5 Chaos vehicle/physics equivalents.
```

Verified car code needing migration:

```text
UE4Project/Plugins/AirSim/Source/Vehicles/Car/CarPawn.h:
  #include "WheeledVehicle.h"
  class ACarPawn : public AWheeledVehicle

UE4Project/Plugins/AirSim/Source/Vehicles/Car/CarPawn.cpp:
  #include "WheeledVehicleMovementComponent4W.h"

UE4Project/Plugins/AirSim/Source/Vehicles/Car/CarPawnSimApi.h:
  #include "WheeledVehicleMovementComponent4W.h"
  #include "vehicles/car/firmwares/physxcar/PhysXCarApi.hpp"
  UWheeledVehicleMovementComponent* movement_

UE4Project/Plugins/AirSim/Source/Vehicles/Car/CarPawnSimApi.cpp:
  #include "PhysXVehicleManager.h"
  PhysXCarApi(...)
  movement_->PVehicle->mWheelsDynData

UE4Project/Plugins/AirSim/Source/Vehicles/Car/CarWheelFront.h:
  #include "VehicleWheel.h"
  class UCarWheelFront : public UVehicleWheel

UE4Project/Plugins/AirSim/Source/Vehicles/Car/CarWheelRear.h:
  #include "VehicleWheel.h"
  class UCarWheelRear : public UVehicleWheel
```

Required update:

```text
Migrate AWheeledVehicle / UWheeledVehicleMovementComponent / UVehicleWheel / PhysXCarApi usage to UE5 Chaos Vehicles equivalents.
Epic's PhysX-to-Chaos guide maps:
  VehicleWheel -> ChaosVehicleWheel
  VehicleMovement -> ChaosWheeledVehicleComponent
  VehicleAnimInstance -> VehicleAnimationInstance
  WheelHandler -> WheelController
  WheeledVehicle -> WheeledVehiclePawn
```

Blueprint/assets needing migration:

```text
TechnionCarPawn
AdsDv_Pawn
front/rear wheel blueprints
vehicle animation blueprints
tire configs
vehicle movement component references
TrainingMap and packaged maps
AirSim camera/lidar/GPS/IMU/GSS sensor attachment paths
```

Contract to preserve for the GX10 ROS/autonomy side:

```text
TCP RPC port: 41451
settings.json vehicle and sensor schema, or a compatibility adapter
ROS topics under /fsds/*
fs_msgs message types
/fsds/control_command behavior
camera, lidar, GPS, IMU, GSS, wheel state, odom, track, TF outputs
```

## Test Log

| Step | Command | Result |
| --- | --- | --- |
| Machine state | `uname -a`, `dpkg --print-architecture` | PASS: Ubuntu 24.04.4 ARM64 |
| GPU | `nvidia-smi` | PASS: GB10 visible, driver 580.142 |
| ROS commands before install | `command -v ros2 colcon rosdep` | FAIL: not installed |
| ROS commands after install | `source /opt/ros/jazzy/setup.bash && ros2 --help` | PASS |
| Git LFS before install | `git lfs install` | FAIL: git-lfs not installed |
| Git LFS after install | `git lfs install && git lfs pull` | BLOCKED: remote LFS budget exceeded |
| FSDS symlink | `ln -sfn ... ~/Formula-Student-Driverless-Simulator` | PASS |
| Old FSDS Linux binary | `file FSOnline/Binaries/Linux/Blocks` | FAIL for GX10 runtime: x86-64 binary |
| ROS apt source staging | download `ros2-apt-source_1.2.0.noble_all.deb` | PASS |
| ROS apt source install | `sudo dpkg -i ...` | PASS after interactive sudo |
| Package install | `sudo apt install ...` | PASS after removing invalid `clang++-16` package name |
| Eigen | manual download to `AirLib/deps/eigen3` | PASS |
| AirLib first build | `timeout 120 bash AirSim/build.sh` | FAIL: forced missing clang-16 |
| Clang availability | `command -v clang-16 clang++-16` | PASS: both available in `/usr/bin` |
| AirLib strict Clang rebuild | `timeout 600 bash AirSim/build.sh` | PASS: used `clang-16` / `clang++-16` |
| AirSim compiler fallback removal | `rg "fallback|falling back|gcc/g\\+\\+|use_compiler_or_fallback" AirSim/build.sh` | PASS: no fallback code remains |
| rosdep init | `sudo rosdep init && rosdep update` | PASS after interactive sudo |
| rosdep install | `rosdep install --from-paths src --ignore-src -r -y` | PASS: all required rosdeps installed |
| rosdep check after strict Clang | `rosdep check --from-paths src --ignore-src` | PASS: all system dependencies satisfied |
| ROS build | `colcon build ...` | PASS: 2 packages finished |
| ROS build after rosdep | `colcon build ...` | PASS: 2 packages finished [0.67s] |
| ROS build after fallback removal | `colcon build ...` | PASS: 2 packages finished [0.59s] |
| Bridge executables | `ros2 pkg executables fsds_ros2_bridge` | PASS |
| ROS demo pub/sub | demo talker/listener | PASS |
| Bridge launch without simulator | `ros2 launch ... host:=127.0.0.1 timeout:=1.0` | PASS for launch wiring; expected RPC connection failure |
| Docker info before group refresh | `docker info` | FAIL in current shell: docker group not active |
| Docker group membership | `getent group docker` | PASS: `docker:x:988:hard` |
| Docker info via active-group workaround | `sg docker -c 'docker info ...'` | PASS: Docker 29.2.1, aarch64 |
| Docker GPU container | `sg docker -c 'docker run --rm --gpus all ... nvidia-smi'` | PASS: container sees NVIDIA GB10 |
| Vulkan tools | `vulkaninfo --summary` | PASS: NVIDIA GB10 visible |
| OpenGL tools | `glxinfo -B` | PASS: NVIDIA GB10 renderer |
| Foxglove | `ros2 run foxglove_bridge foxglove_bridge` | PASS: listening on port 8765 |
| UE5 discovery | `find ... UnrealEditor RunUAT.sh` | INITIAL BLOCKER: UE5 was not installed on GX10 at first discovery |
| Epic GitHub access | `gh auth status`, `gh auth setup-git`, `git ls-remote EpicGames/UnrealEngine` | INITIAL BLOCKER: `gh` was logged in as `LinuxHARD`, but GitHub returned `Repository not found` until the Epic/GitHub account link was fixed |
| UE5.5 toolchain availability | `apt-cache policy clang-18 lld-18 ninja-build dotnet-sdk-8.0` | AVAILABLE: packages exist in Ubuntu Noble ARM64 repos |
| UE5.5 toolchain install | `command -v clang-18 clang++-18 lld-18 ninja dotnet` | PASS: Clang 18, LLD 18, Ninja 1.11.1, and .NET SDK 8.0.126 installed |
| Epic GitHub access after account link | `git ls-remote --tags https://github.com/EpicGames/UnrealEngine.git 'refs/tags/5.5*'` | PASS: tags visible through `5.5.4-release`; selected `5.5.4-release` |
| UE5 Chaos branch | `git branch --show-current` | PASS: current branch is `Converting-to-UE5` |
| UE5 metadata conversion | `.uproject`, `.uplugin`, `AirSim.Build.cs` | PASS: project now targets UE5.5 + Chaos plugin/modules |
| UE5 C++ text conversion | `rg PhysX... UE4Project AirSim/AirLib` | PASS: active text-level PhysX vehicle identifiers removed |
| AirLib after UE5 text conversion | `timeout 600 bash AirSim/build.sh` | PASS |
| ROS after UE5 text conversion | `colcon build ...` | PASS: 2 packages finished [0.56s] |

## Remaining Blockers

1. Git LFS assets cannot be pulled because the remote repository has exceeded its LFS budget.
2. Direct `docker ...` commands require logout/login, reboot, or a shell with docker as an active group. Docker and GPU validation passed through `sg docker -c`.
3. UE5.5 source is installed at `/home/hard/UnrealEngine-5.5`, tag `5.5.4-release`.
4. UE5.5 toolchain packages are installed: `clang-18`, `clang++-18`, `lld-18`, `ninja-build`, and `dotnet-sdk-8.0`.
5. The UE5.5 Chaos LinuxArm64 game target builds and starts, but a packaged/cooked simulator is not available yet.
6. UE5 editor/cooker validation is blocked by missing Autodesk FBX SDK 2020.2 for LinuxArm64. See the 2026-05-18 UE5.5 native execution update below.

## References

- ROS 2 Jazzy Ubuntu packages: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- ROS 2 Jazzy supported platforms: https://docs.ros.org/en/jazzy/Installation.html
- NVIDIA Container Toolkit: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
- UE PhysX-to-Chaos migration: https://dev.epicgames.com/documentation/en-us/unreal-engine/how-to-convert-physx-vehicles-to-chaos-in-unreal-engine
- UE5 LinuxArm64 target note: https://dev.epicgames.com/documentation/en-us/unreal-engine/installed-build-reference-guide-for-unreal-engine
- Cosys-AirSim UE5.5 release/docs: https://github.com/Cosys-Lab/Cosys-AirSim/releases and https://cosys-lab.github.io/Cosys-AirSim/ros_cplusplus/
- Autodesk FBX SDK 2020 Linux runtime library layout: https://help.autodesk.com/cloudhelp/2020/ENU/FBX-Developer-Help/files/getting_started/installing_and_configuring/FBX_Developer_Help_getting_started_installing_and_configuring_configuring_the_fbx_sdk_for_linux_html.html
- Autodesk FBX SDK 2020.2 release notes: https://help.autodesk.com/cloudhelp/2020/ENU/FBX-Developer-Help/files/welcome_to_the_fbx_sdk/what_new/FBX_Developer_Help_welcome_to_the_fbx_sdk_what_new_fbx_sdk_2020_html.html

## UE5.5 Native GX10 Execution Update

Date: 2026-05-18

Branch:

```bash
git branch --show-current
```

Result:

```text
Converting-to-UE5
```

Policy for UE plugins:

```text
Keep UE plugins that are enabled by default. Only disable a plugin or editor feature after verifying that its native LinuxArm64 dependency cannot be provided or built.
```

Unreal Engine source state:

```text
UE source checkout: /home/hard/UnrealEngine-5.5
UE source tag: 5.5.4-release
Target platform on GX10: LinuxArm64 / aarch64
```

UE5 source patches applied locally:

```text
GitDependencies, Setup.sh, SetupDotnet.sh, BuildThirdParty.sh:
  Added ARM64 host detection and system .NET support.

UnrealBuildTool Linux platform/toolchain:
  Uses Arm64 as the default host architecture on this GX10.
  Supports system clang++-18, llvm-ar-18, and llvm-objcopy-18 with -ForceUseSystemCompiler.

ISPCToolChain:
  Honors UE_ISPC_PATH so the local ARM64 ISPC extraction can be used.

UnixPlatformRunnableThread:
  Replaced runtime SIGSTKSZ enum use with fixed crash-handler stack sizes for ARM64 glibc.

Breakpad convert_UTF.c:
  Wrapped implementation in the google_breakpad namespace to match the C++ header.

DumpSyms and BreakpadSymbolEncoder targets:
  Added LinuxArm64 support.

Python3.Build.cs:
  Added LinuxArm64 Python SDK/binary path support.

USDCore / UnrealUSDWrapper:
  Added LinuxArm64 Python and Intel TBB path support.
  Patched editor USD library lookup from hardcoded x86_64 to aarch64 on ARM64.

USD BuildForLinux.sh:
  Added ARM64 host detection, ARM64 Boost suffix, LinuxArm64 Python path, clang-18 toolchain, and aarch64 USD library output paths.
```

Local third-party ARM64 builds staged:

```text
Python 3.11.8:
  Built locally under /tmp/ue-python-3.11.8-arm64.
  Staged to Engine/Source/ThirdParty/Python3/LinuxArm64 and Engine/Binaries/ThirdParty/Python3/LinuxArm64.

Intel TBB:
  Built ARM64 shared libraries from UE bundled source.
  Staged to Engine/Binaries/ThirdParty/Intel/TBB/LinuxArm64.

rpclib:
  Rebuilt for ARM64 with UE libc++ headers.
  Copied to AirSim/AirLib/deps/rpclib/lib/librpc.a and UE4Project/Plugins/AirSim/Source/AirLib/deps/rpclib/lib/librpc.a.

Ogg/Vorbis:
  Rebuilt ARM64 archives with clang-18 and without finite-math-only flags to avoid glibc finite-symbol link failures.

OpenUSD 24.05:
  Built successfully for ARM64.
  Staged 49 ARM64 USD shared libraries under Engine/Plugins/Runtime/USDCore/Source/ThirdParty/Linux/bin/aarch64-unknown-linux-gnueabi.
  Replaced the local Linux USDCore Python/resource payload with the ARM64 payload generated by the build script.
```

Temporary build environment required for current UE commands:

```bash
UE_USE_SYSTEM_DOTNET=1 \
UE_ISPC_PATH=/tmp/ue-ispc-arm64/usr/bin/ispc \
LD_LIBRARY_PATH=/tmp/ue-ispc-arm64/usr/lib/aarch64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
```

Permanent package still needed when sudo is available:

```bash
sudo apt install -y ispc libllvm17t64
```

UE5 Chaos repo conversion performed:

```text
UE4Project/FSOnline.uproject:
  EngineAssociation is now 5.5.
  ChaosVehiclesPlugin is enabled.
  DisableEnginePluginsByDefault is not set.

UE4Project/Plugins/AirSim/AirSim.uplugin:
  PhysXVehicles removed.
  ChaosVehiclesPlugin enabled.

UE4Project/Plugins/AirSim/Source/AirSim.Build.cs:
  Removed PhysXVehicles, PhysXVehicleLib, PhysX, and APEX.
  Added ChaosVehicles, ChaosVehiclesCore, Chaos, and ChaosCore.
  Added LinuxArm64 handling for rpclib.
  Uses /usr/include/eigen3 on Linux/LinuxArm64 because the bundled Eigen copy is not C++20-clean.

AirSim camera director:
  Renamed CameraDirector to AirSimCameraDirector to avoid UE5 GameplayCameras ACameraDirector collision.

Car runtime:
  Replaced AWheeledVehicle with AWheeledVehiclePawn.
  Replaced UWheeledVehicleMovementComponent with UChaosWheeledVehicleMovementComponent.
  Replaced UVehicleWheel with UChaosVehicleWheel.
  Removed PhysXCarApi and added ChaosCarApi.
  Removed direct PhysX PVehicle wheel-state access.
```

Native game build command:

```bash
UE_USE_SYSTEM_DOTNET=1 \
UE_ISPC_PATH=/tmp/ue-ispc-arm64/usr/bin/ispc \
LD_LIBRARY_PATH=/tmp/ue-ispc-arm64/usr/lib/aarch64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH} \
/home/hard/UnrealEngine-5.5/Engine/Build/BatchFiles/Linux/Build.sh Blocks LinuxArm64 Development \
  -Project=/home/hard/Desktop/Formula-Student-Driverless-Simulator/UE4Project/FSOnline.uproject \
  -ForceUseSystemCompiler -buildubt
```

Result:

```text
PASS. Blocks LinuxArm64 Development builds.
Post-link Breakpad symbol generation now works after building LinuxArm64 DumpSyms and BreakpadSymbolEncoder.
Generated binary: UE4Project/Binaries/LinuxArm64/Blocks
```

Runtime layout fix:

```bash
ln -sfn /home/hard/UnrealEngine-5.5/Engine /home/hard/Desktop/Formula-Student-Driverless-Simulator/Engine
```

Reason:

```text
The raw Blocks binary resolves ../../../Engine relative to UE4Project/Binaries/LinuxArm64.
Without this symlink it cannot find Engine/Content/Internationalization.
```

Native runtime smoke test:

```bash
timeout 90 UE4Project/Binaries/LinuxArm64/Blocks \
  -nullrhi -nosound -unattended -stdout -FullStdOutLogOutput -log
```

Result:

```text
PARTIAL PASS.
The ARM64 UE5 binary starts, mounts the project AirSim plugin, initializes the engine, and reports:
  Platform=LinuxArm64
  Architecture=arm64
  Physics initialised using underlying interface: Chaos

Expected remaining failure:
  The non-editor game binary cannot load uncooked UE assets and crashes while loading /Engine/EngineMaterials/WorldGridMaterial.

Conclusion:
  A cooker/editor path is still required before the GX10 can run a full packaged simulator.
```

Editor/cooker build command:

```bash
UE_USE_SYSTEM_DOTNET=1 \
UE_ISPC_PATH=/tmp/ue-ispc-arm64/usr/bin/ispc \
LD_LIBRARY_PATH=/tmp/ue-ispc-arm64/usr/lib/aarch64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH} \
/home/hard/UnrealEngine-5.5/Engine/Build/BatchFiles/Linux/Build.sh BlocksEditor Linux Development \
  -Project=/home/hard/Desktop/Formula-Student-Driverless-Simulator/UE4Project/FSOnline.uproject \
  -ForceUseSystemCompiler -buildubt
```

Result:

```text
BLOCKED.
USDCore ARM64 dependency is fixed.
The next editor/cooker blocker is FBX:
  FBX SDK not found in ../Binaries/ThirdParty/FBX/2020.2/Linux/aarch64-unknown-linux-gnueabi/
```

FBX verification:

```bash
rg -n "FBX/2020.2/Linux" /home/hard/UnrealEngine-5.5/Engine/Build/Commit.gitdeps.xml
find /home/hard/UnrealEngine-5.5/Engine/Binaries/ThirdParty/FBX/2020.2/Linux -maxdepth 2 -type f
```

Result:

```text
Epic UE5.5 gitdeps provides only:
  Engine/Binaries/ThirdParty/FBX/2020.2/Linux/x86_64-unknown-linux-gnu/libfbxsdk.so

No LinuxArm64/aarch64 FBX SDK binary is present in the UE5.5 dependency manifest.
The local x86_64 libfbxsdk.so cannot be linked into a LinuxArm64 editor.
```

External source check:

```text
Autodesk's FBX SDK 2020 Linux documentation lists Linux runtime libraries for gcc x86 and gcc x64, not Linux ARM64/aarch64.
Autodesk's FBX SDK 2020.2 release notes mention Apple M1 universal binaries, not Linux ARM64 binaries.
```

Current native-GX10 status:

```text
ROS 2 Jazzy bridge pipeline: PASS.
UE5.5 Chaos C++ game target for LinuxArm64: PASS.
UE5.5 native runtime smoke test: PARTIAL PASS, starts and initializes Chaos but needs cooked assets.
UE5.5 editor/cooker for LinuxArm64: BLOCKED by missing Autodesk FBX SDK LinuxArm64 binary.
Default UE plugins: still enabled; no broad DisableEnginePluginsByDefault fallback was applied.
```

Decision needed before editor/cooker can proceed:

```text
Preferred:
  Provide a real Autodesk/Epic-compatible LinuxArm64 libfbxsdk.so for FBX SDK 2020.2, if one exists outside Epic gitdeps.

Allowed only because no LinuxArm64 FBX SDK binary has been found:
  Disable or compile out FBX-dependent editor/importer paths for the GX10 editor/cooker build.
  This would remove FBX import/export functionality on the GX10 editor build, but should not affect the already-imported FSDS runtime assets if cooking succeeds.
```
