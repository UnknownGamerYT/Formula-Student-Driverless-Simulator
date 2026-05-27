# UE4 To UE5 ROS 2 Jazzy Migration Plan

Created: 2026-05-18

This document describes what must change to migrate Formula Student Driverless Simulator from Unreal Engine 4.27 / PhysX to Unreal Engine 5.5 with Chaos Vehicles while keeping the ASUS Ascent GX10 ROS 2 Jazzy pipeline working.

The migration should be done on a dedicated branch. Do not convert the current UE4 project in-place on `master` until a UE5 simulator has passed the ROS bridge acceptance tests below.

## Goal

- Produce a UE5.5 + Chaos Vehicles FSDS simulator that preserves the existing ROS 2 Jazzy autonomy interface.
- Keep the GX10 as the native ROS 2 / autonomy machine.
- Use an x86_64 UE5 simulator host first, then attempt a GX10-native LinuxArm64 simulator after the UE5 port is proven.
- Preserve the existing AirSim-style TCP RPC interface on port `41451` so `fsds_ros2_bridge` and `fs_msgs` do not need to be redesigned as part of this migration.
- Do not keep a UE4, PhysX, or legacy packaged simulator fallback path for the migrated runtime.

Execution path:

```text
Phase 1: x86_64 UE5.5 + Chaos simulator host + GX10 ROS 2 Jazzy bridge over TCP 41451
Phase 2: GX10-native UE5.5 + Chaos LinuxArm64 simulator build/package
```

UE5 AirSim base constraint:

```text
Cosys-AirSim v3.3 for Unreal Engine 5.5 may be used as the AirSim base only if the final FSDS vehicle runtime is UE5 Chaos-based and preserves the FSDS RPC contract.
```

## Current UE4 / PhysX State

Verified project association:

```text
UE4Project/FSOnline.uproject:
  EngineAssociation: 4.27
```

Verified plugin dependency:

```text
UE4Project/Plugins/AirSim/AirSim.uplugin:
  Plugins:
    PhysXVehicles enabled
```

Verified module dependencies:

```text
UE4Project/Plugins/AirSim/Source/AirSim.Build.cs:
  PhysXVehicles
  PhysXVehicleLib
  PhysX
  APEX
```

Additional UE4 / PhysX config:

```text
UE4Project/Config/DefaultEngine.ini:
  PhysXTreeRebuildRate=10
```

Code that currently blocks a direct UE5 build:

```text
UE4Project/Plugins/AirSim/Source/Vehicles/Car/CarPawn.h:
  AWheeledVehicle

UE4Project/Plugins/AirSim/Source/Vehicles/Car/CarPawn.cpp:
  WheeledVehicleMovementComponent4W.h
  GetVehicleMovement()->GetForwardSpeed()
  GetVehicleMovement()->GetCurrentGear()

UE4Project/Plugins/AirSim/Source/Vehicles/Car/CarPawnSimApi.h:
  WheeledVehicleMovementComponent4W.h
  PhysXCarApi.hpp
  UWheeledVehicleMovementComponent*

UE4Project/Plugins/AirSim/Source/Vehicles/Car/CarPawnSimApi.cpp:
  PhysXVehicleManager.h
  PhysXCarApi(...)
  movement_->PVehicle->mWheelsDynData
  SetThrottleInput / SetSteeringInput / SetBrakeInput

UE4Project/Plugins/AirSim/Source/Vehicles/Car/CarWheelFront.h:
  UVehicleWheel

UE4Project/Plugins/AirSim/Source/Vehicles/Car/CarWheelRear.h:
  UVehicleWheel

UE4Project/Plugins/AirSim/Source/Vehicles/Car/CarWheelFront.cpp:
  UTireConfig
  Vehicle_FrontTireConfig

UE4Project/Plugins/AirSim/Source/Vehicles/Car/CarWheelRear.cpp:
  UTireConfig
  Vehicle_BackTireConfig

AirSim/AirLib/include/vehicles/car/firmwares/physxcar/PhysXCarApi.hpp:
  PhysXCarApi
```

Important FSDS assets to preserve or migrate:

```text
UE4Project/Plugins/AirSim/Content/VehicleAdv/Cars/TechnionCar/TechnionCarPawn.uasset
UE4Project/Plugins/AirSim/Content/VehicleAdv/Cars/TechnionCar/FormulaAnim.uasset
UE4Project/Plugins/AirSim/Content/VehicleAdv/Cars/TechnionCar/FormulaMesh_PhysicsAsset.uasset
UE4Project/Plugins/AirSim/Content/VehicleAdv/Cars/AdsDv/AdsDv_Pawn.uasset
UE4Project/Plugins/AirSim/Content/VehicleAdv/Cars/AdsDv/AdsDv_Anim.uasset
UE4Project/Plugins/AirSim/Content/VehicleAdv/Cars/AdsDv/wheels/*
UE4Project/Plugins/AirSim/Content/VehicleAdv/WheelData/*
UE4Project/Content/FormulaStudentAssets/*
UE4Project/Content/TrainingMap.umap
UE4Project/Content/Acceleration.umap
UE4Project/Content/Skidpad.umap
UE4Project/Content/Competition/*.umap
```

## Required UE5 Chaos Migration Strategy

Use Unreal Engine 5.5 with Chaos Vehicles as the only target simulator runtime. Cosys-AirSim v3.3 can be used as the UE5-compatible AirSim plugin base, but it is not a fallback around the Chaos migration: the final FSDS car runtime must use Chaos Vehicles.

Why this is the required route:

- The bundled FSDS AirSim plugin is UE4.27 and PhysX-based.
- UE5 vehicle support is Chaos-based, not PhysXVehicles-based.
- Cosys-AirSim already tracks UE5 releases and has UE5.5 plugin releases.
- Starting from a UE5-maintained AirSim fork may reduce engine API migration work, but all FSDS vehicle behavior must still be migrated to Chaos.

Required work:

1. Create a migration branch, for example `ue5-chaos`.
2. Keep the directory name `UE4Project` during the first port unless there is a separate reason to rename it. Existing scripts and docs refer to this path.
3. Install or build Unreal Engine 5.5 on the first x86_64 simulator host.
4. Add a UE5.5-compatible AirSim plugin base, preferably Cosys-AirSim v3.3.
5. Port FSDS-specific Unreal code into the UE5 plugin/project:
   - car pawn behavior
   - referee state and mission state
   - custom map loader
   - Formula Student maps and cone assets
   - sensor placement and defaults
   - settings loading behavior
   - RPC methods needed by the ROS bridge
6. Migrate the vehicle from PhysX classes/assets to Chaos Vehicles.
7. Remove PhysX runtime dependencies from project metadata, plugin metadata, build rules, C++ code, and vehicle assets.
8. Keep the existing ROS 2 bridge unchanged until the simulator passes the acceptance tests.

Do not switch to the native Cosys-AirSim ROS 2 wrapper as part of the first milestone. That wrapper may be useful later, but using it now would change the ROS topic/API surface and hide simulator migration issues.

## No Fallback Policy

The migrated simulator has one accepted runtime target: UE5.5 with Chaos Vehicles.

- Do not ship or accept UE4.27 as the simulator runtime after this migration.
- Do not keep `PhysXVehicles`, `PhysXVehicleLib`, `PhysX`, or `APEX` in the UE5 runtime dependencies.
- Do not keep `AWheeledVehicle`, `UWheeledVehicleMovementComponent`, `UVehicleWheel`, `UTireConfig`, `PhysXVehicleManager`, or `PhysXCarApi` in the migrated car runtime.
- Do not use the old x86-64 packaged FSDS binary as a success criterion.
- Do not switch the ROS contract to Cosys-AirSim ROS 2 topics as a way to bypass FSDS RPC compatibility.

Mandatory UE5 Chaos replacements:

1. Update `UE4Project/FSOnline.uproject` to the chosen UE5 association only after the project is copied or the migration branch is ready.
2. Remove `PhysXVehicles` from `AirSim.uplugin`.
3. Replace `PhysXVehicles`, `PhysXVehicleLib`, `PhysX`, and `APEX` in `AirSim.Build.cs` with UE5 Chaos vehicle/physics module dependencies.
4. Enable UE5's Chaos Vehicles plugin.
5. Replace UE4 vehicle C++ classes:
   - `AWheeledVehicle` -> UE5 Chaos wheeled vehicle pawn class
   - `UWheeledVehicleMovementComponent` -> `UChaosWheeledVehicleMovementComponent`
   - `UVehicleWheel` -> `UChaosVehicleWheel`
   - `UTireConfig` usage -> Chaos wheel friction / physical material setup
6. Remove direct PhysX access such as `movement_->PVehicle->mWheelsDynData`.
7. Replace `PhysXCarApi` with a Chaos-compatible car API implementation while preserving the RPC structures consumed by the ROS bridge.
8. Rebuild AirLib and the UE plugin for the target platform.

## Phase 1: UE5 Chaos x86_64 Simulator Host

Phase 1 should make the simulator run on a known-supported x86_64 machine first. The GX10 remains the ROS 2 Jazzy machine and connects over the network.

Supported first host choices:

```text
Windows 11 x86_64 + Unreal Engine 5.5 + Visual Studio 2022
Ubuntu x86_64 + Unreal Engine 5.5 + clang/toolchain required by that engine build
```

Network contract:

```text
Simulator host: runs UE5 FSDS and listens on TCP 41451
GX10: runs ROS 2 Jazzy, fsds_ros2_bridge, autonomy stack, Foxglove bridge
```

Host firewall requirement:

```bash
# Linux x86_64 simulator host
sudo ufw allow 41451/tcp

# Windows x86_64 simulator host, run in an elevated PowerShell
New-NetFirewallRule -DisplayName "FSDS AirSim RPC 41451" -Direction Inbound -Action Allow -Protocol TCP -LocalPort 41451
```

GX10 connectivity check:

```bash
nc -vz <SIM_HOST_IP> 41451
```

GX10 bridge command:

```bash
source /opt/ros/jazzy/setup.bash
source ros2/install/setup.bash
ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py host:=<SIM_HOST_IP> timeout:=120.0
```

## Phase 2: GX10 Native LinuxArm64 UE5 Chaos Simulator

Only begin Phase 2 after Phase 1 passes the bridge acceptance tests. The GX10 is Ubuntu 24.04 ARM64 with NVIDIA GB10, so prebuilt x86_64 simulator binaries and x86_64 plugin binaries cannot run natively.

LinuxArm64 requirements:

- UE5 source or installed build that includes LinuxArm64 target support.
- The UE5.5 Chaos AirSim plugin built from source for LinuxArm64.
- AirLib, rpclib, and any third-party native libraries built for `aarch64`.
- Packaged simulator binary verified as ARM64 with `file`.

Engine build discovery:

```bash
export UE5_ROOT=/path/to/UnrealEngine-5.5
"$UE5_ROOT/Engine/Build/BatchFiles/RunUAT.sh" BuildGraph \
  -script="$UE5_ROOT/Engine/Build/InstalledEngineBuild.xml" \
  -listonly | rg "Make Installed Build|LinuxArm64"
```

Installed build template:

```bash
export UE5_ROOT=/path/to/UnrealEngine-5.5-source
"$UE5_ROOT/Engine/Build/BatchFiles/RunUAT.sh" BuildGraph \
  -target="Make Installed Build Linux" \
  -script="$UE5_ROOT/Engine/Build/InstalledEngineBuild.xml" \
  -set:WithLinux=true \
  -set:WithLinuxArm64=true \
  -set:WithWin64=false \
  -set:WithMac=false
```

LinuxArm64 package template:

```bash
export UE5_ROOT=/path/to/UE5-installed-or-source
"$UE5_ROOT/Engine/Build/BatchFiles/RunUAT.sh" BuildCookRun \
  -project="$PWD/UE4Project/FSOnline.uproject" \
  -noP4 \
  -build \
  -cook \
  -stage \
  -pak \
  -archive \
  -archivedirectory="$PWD/UE5Builds/LinuxArm64" \
  -clientconfig=Development \
  -platform=LinuxArm64
```

Native binary checks:

```bash
file UE5Builds/LinuxArm64/**/Binaries/Linux/* 2>/dev/null
ldd UE5Builds/LinuxArm64/**/Binaries/Linux/* 2>/dev/null
```

Expected binary architecture:

```text
aarch64 / ARM64, not x86-64
```

## UE4 To UE5 Code Migration Checklist

Project and plugin metadata:

- Update `UE4Project/FSOnline.uproject` `EngineAssociation` to `5.5` only on the migration branch.
- Keep the `Blocks` runtime module unless the UE5 conversion requires a target/module rename.
- Remove `PhysXVehicles` from `UE4Project/Plugins/AirSim/AirSim.uplugin`.
- Remove `PhysXTreeRebuildRate` from `UE4Project/Config/DefaultEngine.ini` if UE5 reports it as invalid.
- Enable the UE5 Chaos Vehicles plugin in the project or plugin dependencies.

Build rules:

- Replace `PhysXVehicles`, `PhysXVehicleLib`, `PhysX`, and `APEX` dependencies in `AirSim.Build.cs`.
- Add the UE5 Chaos vehicle/physics modules required by the UE5 simulator plugin path.
- Keep `Core`, `CoreUObject`, `Engine`, `HTTP`, `InputCore`, `ImageWrapper`, `RenderCore`, `RHI`, `PhysicsCore`, and `Landscape` unless UE5 compile errors prove a change is needed.
- Rebuild AirLib and copy the generated AirLib output into `UE4Project/Plugins/AirSim/Source/AirLib` before compiling the UE plugin.

Vehicle C++:

- Migrate `CarPawn.h/.cpp` from UE4 wheeled vehicle classes to UE5 Chaos vehicle classes.
- Migrate `CarPawnSimApi.h/.cpp` from PhysX movement internals to Chaos movement APIs.
- Replace direct wheel dynamic data reads with a Chaos-compatible source for RPM, rotation angle, steering angle, and wheel state.
- Replace `PhysXCarApi` with a Chaos-compatible car API while keeping the existing AirSim car control/state structures used by RPC.
- Keep `setCarControls`, `getCarState`, `simGetWheelStates`, and sensor APIs source-compatible with the GX10 ROS bridge until acceptance passes.

World and referee API:

- Preserve `WorldSimApi::getSettingsString`.
- Preserve `WorldSimApi::getRefereeState`.
- Preserve custom map loading behavior and mission/referee state behavior used by the existing ROS bridge.

## Blueprint And Asset Migration Checklist

Open the converted project in UE5.5 and migrate assets in this order:

1. Enable Chaos Vehicles.
2. Convert or recreate wheel blueprints/classes:
   - `FormulaFrontWheel`
   - `FormulaBackWheel`
   - `AdsDvFrontWheel`
   - `AdsDvBackWheel`
3. Convert or replace tire config assets:
   - `Vehicle_FrontTireConfig`
   - `Vehicle_BackTireConfig`
   - `FormulaFrontTire`
   - `FormulaBackTire`
   - `Slippery`
   - `NonSlippery`
4. Convert car pawns:
   - `TechnionCarPawn`
   - `AdsDv_Pawn`
   - `ReferenceCarPawn`
   - `SuvCarPawn`
5. Update vehicle movement component references to Chaos components.
6. Verify skeletal meshes, physics assets, wheel bone names, center of mass, collision bodies, and animation blueprints.
7. Preserve maps and Formula Student assets unless UE5 reports broken references:
   - `TrainingMap`
   - `Acceleration`
   - `Skidpad`
   - `CompetitionMap*`
   - `FormulaStudentAssets/*`

Do not delete UE4 assets during the first conversion pass. Mark broken references in a migration log and replace them only after the simulator compiles.

## ROS 2 Jazzy Contract

The UE5 simulator must preserve this external contract.

RPC:

```text
TCP port: 41451
Vehicle name: FSCar
Settings file: ~/Formula-Student-Driverless-Simulator/settings.json
```

Required `settings.json` values:

```text
PawnPaths.DefaultCar:
  Class'/AirSim/VehicleAdv/Cars/TechnionCar/TechnionCarPawn.TechnionCarPawn_C'

Vehicles.FSCar.Cameras:
  cam1
  cam2

Vehicles.FSCar.Sensors:
  Imu
  Gps
  Lidar1
  Lidar2
  GSS
```

Required RPC methods used by `fsds_ros2_bridge`:

```text
getSettingsString
confirmConnection
enableApiControl
setCarControls
getCarState
getGpsData
getImuData
getGroundSpeedSensorData
simGetWheelStates
getLidarData
simGetImages
getRefereeState
```

ROS packages to keep working:

```text
ros2/src/fs_msgs
ros2/src/fsds_ros2_bridge
```

Important ROS topics/messages:

```text
/fsds/control_command                 fs_msgs/msg/ControlCommand
/fsds/gps                             sensor_msgs/msg/NavSatFix
/fsds/imu                             sensor_msgs/msg/Imu
/fsds/gss                             geometry/speed-derived bridge output
/fsds/wheel_states                    fs_msgs/msg/WheelStates
/fsds/track                           fs_msgs/msg/Track
/fsds/camera/cam1/*                   camera output from simGetImages
/fsds/camera/cam2/*                   camera output from simGetImages
```

The ROS bridge source should remain unchanged for the first UE5 simulator acceptance run. If a Cosys-AirSim RPC shape differs, adapt the UE5 simulator side or add a compatibility shim before changing ROS topics.

## Build And Run Commands

Linux x86_64 UE5 editor command:

```bash
export UE5_ROOT=/path/to/UnrealEngine-5.5
"$UE5_ROOT/Engine/Binaries/Linux/UnrealEditor" \
  "$PWD/UE4Project/FSOnline.uproject" \
  -log
```

Windows x86_64 UE5 editor command:

```powershell
$env:UE5_ROOT="C:\Program Files\Epic Games\UE_5.5"
& "$env:UE5_ROOT\Engine\Binaries\Win64\UnrealEditor.exe" `
  "$PWD\UE4Project\FSOnline.uproject" `
  -log
```

Linux x86_64 package command:

```bash
export UE5_ROOT=/path/to/UnrealEngine-5.5
"$UE5_ROOT/Engine/Build/BatchFiles/RunUAT.sh" BuildCookRun \
  -project="$PWD/UE4Project/FSOnline.uproject" \
  -noP4 \
  -build \
  -cook \
  -stage \
  -pak \
  -archive \
  -archivedirectory="$PWD/UE5Builds/Linux" \
  -clientconfig=Development \
  -platform=Linux
```

Windows x86_64 package command:

```powershell
$env:UE5_ROOT="C:\Program Files\Epic Games\UE_5.5"
& "$env:UE5_ROOT\Engine\Build\BatchFiles\RunUAT.bat" BuildCookRun `
  -project="$PWD\UE4Project\FSOnline.uproject" `
  -noP4 `
  -build `
  -cook `
  -stage `
  -pak `
  -archive `
  -archivedirectory="$PWD\UE5Builds\Windows" `
  -clientconfig=Development `
  -platform=Win64
```

Run the packaged simulator with logging enabled. The final binary name may still be `Blocks` because the current runtime module is named `Blocks`.

```bash
./<PACKAGED_FSDS_BINARY> -windowed -ResX=1280 -ResY=720 -log
```

GX10 network and bridge test:

```bash
nc -vz <SIM_HOST_IP> 41451

source /opt/ros/jazzy/setup.bash
source ros2/install/setup.bash
ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py host:=<SIM_HOST_IP> timeout:=120.0
```

Foxglove bridge:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run foxglove_bridge foxglove_bridge
```

Control command smoke test:

```bash
source /opt/ros/jazzy/setup.bash
source ros2/install/setup.bash
ros2 topic pub --once /fsds/control_command fs_msgs/msg/ControlCommand \
  "{header: {frame_id: fsds/FSCar}, throttle: 0.2, steering: 0.0, brake: 0.0}"
```

Topic checks:

```bash
ros2 topic list | rg '^/fsds'
ros2 topic hz /fsds/imu
ros2 topic hz /fsds/gps
ros2 topic hz /fsds/wheel_states
ros2 topic echo --once /fsds/track
```

## Acceptance Tests

Documentation acceptance:

- This file defines one simulator target: UE5.5 with Chaos Vehicles.
- This file explicitly rejects UE4, PhysX, and legacy packaged simulator fallback paths for the migrated runtime.
- Cosys-AirSim v3.3 for UE5.5 is marked only as an allowed UE5 AirSim base, not as an alternative to Chaos.
- The current UE4.27 / PhysX blockers are listed with concrete repo paths.
- Phase 1 documents an x86_64 UE5 Chaos simulator host first.
- Phase 2 documents GX10-native LinuxArm64 UE5 Chaos after Phase 1.

UE5 Chaos editor acceptance:

- UE5.5 opens the converted project.
- No missing module errors remain.
- No missing parent class errors remain for car pawns.
- No missing wheel class or vehicle movement component errors remain.
- Maps open without fatal missing asset errors.

UE5 Chaos package acceptance:

- Package succeeds for the Phase 1 x86_64 host.
- Packaged simulator starts with `-log`.
- Simulator listens on TCP `41451`.
- Simulator loads `FSCar` with `cam1`, `cam2`, `Imu`, `Gps`, `Lidar1`, `Lidar2`, and `GSS`.

GX10 ROS bridge acceptance:

- `nc -vz <SIM_HOST_IP> 41451` passes from the GX10.
- `ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py host:=<SIM_HOST_IP> timeout:=120.0` logs a successful simulator connection.
- `/fsds/*` topics publish.
- Camera topics publish images from `cam1` and `cam2`.
- IMU, GPS, GSS, lidar, wheel states, odom, and track data publish.
- Publishing `/fsds/control_command` changes throttle/steering/brake behavior in the simulator.
- Foxglove bridge starts on port `8765` and can inspect the ROS topics.

GX10-native LinuxArm64 acceptance:

- UE5 LinuxArm64 package succeeds.
- `file` reports the packaged simulator binary as ARM64/aarch64.
- The binary starts on the GX10.
- The simulator sees the NVIDIA GB10 renderer.
- The simulator listens on TCP `41451`.
- The same ROS bridge acceptance tests pass with `host:=127.0.0.1` or the GX10 LAN IP.

## Execution Log

Started on the GX10 on the UE5 conversion branch. The current branch name is `Converting-to-UE5`.

Local setup discovery:

```text
Initial discovery: UE5 editor / RunUAT.sh was not found under /home/hard, /opt, /usr/local, or /mnt.
GitHub CLI: logged in as `LinuxHARD`; Git has been configured through `gh auth setup-git`.
EpicGames/UnrealEngine git access: working after Epic/GitHub account linking.
UE5.5 Linux compiler requirement from Epic docs: clang 18.1.0 family.
Ubuntu 24.04 ARM64 packages available: clang-18, lld-18, ninja-build, dotnet-sdk-8.0.
Installed UE5.5 build tools: clang-18, clang++-18, lld-18, ninja-build, dotnet-sdk-8.0.
Selected Unreal Engine source tag: 5.5.4-release.
UE5.5 source is now installed at /home/hard/UnrealEngine-5.5.
```

First repo-side UE5 Chaos conversion pass completed:

```text
UE4Project/FSOnline.uproject:
  EngineAssociation changed from 4.27 to 5.5.
  ChaosVehiclesPlugin enabled.

UE4Project/Plugins/AirSim/AirSim.uplugin:
  PhysXVehicles dependency replaced with ChaosVehiclesPlugin.

UE4Project/Plugins/AirSim/Source/AirSim.Build.cs:
  PhysXVehicles, PhysXVehicleLib, PhysX, and APEX removed.
  ChaosVehicles, ChaosVehiclesCore, Chaos, and ChaosCore added.

UE4Project/Config/DefaultEngine.ini:
  PhysXTreeRebuildRate removed.

AirSim/AirLib/include/vehicles/car/firmwares:
  PhysXCarApi removed.
  ChaosCarApi added.

UE4Project/Plugins/AirSim/Source/Vehicles/Car:
  CarPawn moved from AWheeledVehicle to AWheeledVehiclePawn.
  Wheel classes moved from UVehicleWheel to UChaosVehicleWheel.
  CarPawnSimApi now uses UChaosWheeledVehicleMovementComponent and ChaosCarApi.
  Direct PVehicle / PVehicleDrive PhysX access removed.
  Wheel state extraction now uses Chaos wheel accessors.
```

Validation performed after the first pass:

```text
JSON validation passed for FSOnline.uproject and AirSim.uplugin.
Text search found no remaining active PhysX vehicle identifiers in UE4Project or AirSim/AirLib.
AirLib rebuild passed with clang-16/clang++-16.
ROS 2 workspace rebuild passed: fs_msgs and fsds_ros2_bridge.
```

Historical source checkout command:

```bash
git clone --branch 5.5.4-release --single-branch https://github.com/EpicGames/UnrealEngine.git ~/UnrealEngine-5.5
```

Initial upstream build command template:

```bash
cd ~/UnrealEngine-5.5
./Setup.sh
./GenerateProjectFiles.sh
make UnrealEditor
```

## Known Blockers

- The released FSDS Linux runtime is x86-64 and cannot run natively on the ARM64 GX10.
- UE5.5 source is installed at `/home/hard/UnrealEngine-5.5`, tag `5.5.4-release`.
- GitHub CLI is authenticated as `LinuxHARD`, and EpicGames/UnrealEngine access is now working.
- UE5.5 toolchain packages are installed: `clang-18`, `clang++-18`, `lld-18`, `ninja-build`, and `dotnet-sdk-8.0`.
- Git LFS assets may be unavailable until the remote repository LFS budget is fixed.
- Precompiled x86_64 UE5 plugins or native libraries cannot be used for GX10-native LinuxArm64.
- The UE5.5 Chaos LinuxArm64 game target builds and starts, but a cooked/package simulator is not available yet.
- UE5.5 editor/cooker is blocked by missing Autodesk FBX SDK 2020.2 for LinuxArm64.
- Car Blueprint, wheel Blueprint, tire/physical material, animation Blueprint, and physics asset migration still require the UE5.5 editor.
- Cosys-AirSim may not include FSDS-specific `getRefereeState` behavior, so that compatibility must be ported or shimmed.
- The first acceptance run must keep the existing ROS 2 topic/message contract stable.

## References

- Epic PhysX-to-Chaos vehicle migration: https://dev.epicgames.com/documentation/en-us/unreal-engine/how-to-convert-physx-vehicles-to-chaos-in-unreal-engine
- Epic installed-build reference, including LinuxArm64 target options: https://dev.epicgames.com/documentation/en-us/unreal-engine/installed-build-reference-guide-for-unreal-engine
- Cosys-AirSim releases: https://github.com/Cosys-Lab/Cosys-AirSim/releases
- Cosys-AirSim ROS 2 C++ wrapper docs: https://cosys-lab.github.io/Cosys-AirSim/ros_cplusplus/
- Current GX10 setup log: `SETUP_ROS2_JAZZY_GX10_UE5.md`
- Existing Windows / WSL2 setup log: `SETUP_ROS2_JAZZY_WINDOWS.md`
- Autodesk FBX SDK 2020 Linux runtime library layout: https://help.autodesk.com/cloudhelp/2020/ENU/FBX-Developer-Help/files/getting_started/installing_and_configuring/FBX_Developer_Help_getting_started_installing_and_configuring_configuring_the_fbx_sdk_for_linux_html.html
- Autodesk FBX SDK 2020.2 release notes: https://help.autodesk.com/cloudhelp/2020/ENU/FBX-Developer-Help/files/welcome_to_the_fbx_sdk/what_new/FBX_Developer_Help_welcome_to_the_fbx_sdk_what_new_fbx_sdk_2020_html.html

## GX10 Native UE5.5 Execution Update

Date: 2026-05-18

Default-plugin policy:

```text
Keep UE plugins enabled by default. Do not use DisableEnginePluginsByDefault as a broad workaround.
Only disable or compile out a plugin/editor feature after verifying that its required LinuxArm64 dependency cannot be provided or built.
```

Current implementation status:

```text
Branch: Converting-to-UE5
UE source: /home/hard/UnrealEngine-5.5
UE tag: 5.5.4-release
GX10 target: LinuxArm64 / aarch64
```

Implemented UE5 Chaos C++ migration items:

```text
UE4Project/FSOnline.uproject:
  EngineAssociation set to 5.5.
  ChaosVehiclesPlugin enabled.

UE4Project/Plugins/AirSim/AirSim.uplugin:
  PhysXVehicles removed.
  ChaosVehiclesPlugin enabled.

UE4Project/Plugins/AirSim/Source/AirSim.Build.cs:
  Removed PhysXVehicles, PhysXVehicleLib, PhysX, and APEX.
  Added ChaosVehicles, ChaosVehiclesCore, Chaos, and ChaosCore.
  Added LinuxArm64 handling for rpclib.

Car runtime:
  AWheeledVehicle -> AWheeledVehiclePawn.
  UWheeledVehicleMovementComponent -> UChaosWheeledVehicleMovementComponent.
  UVehicleWheel -> UChaosVehicleWheel.
  PhysXCarApi removed.
  ChaosCarApi added.
  Direct PhysX wheel-state access removed.

UE5 API cleanup:
  CameraDirector renamed to AirSimCameraDirector to avoid UE5 GameplayCameras naming collision.
  UE4 input delegate typedefs updated for UE5.
  RHI buffer locking updated for UE5.
  nlohmann json allocator calls updated for C++20.
```

Validation status:

```text
AirLib strict clang-16 build: PASS.
ROS 2 Jazzy colcon build: PASS.
Blocks LinuxArm64 Development: PASS.
Breakpad symbol generation for Blocks: PASS.
Native Blocks ARM64 smoke test: PARTIAL PASS.
```

Native smoke-test details:

```text
Command:
  UE4Project/Binaries/LinuxArm64/Blocks -nullrhi -nosound -unattended -stdout -FullStdOutLogOutput -log

Observed:
  The binary starts on GX10.
  Project AirSim plugin mounts.
  Chaos initializes.
  Log reports Platform=LinuxArm64 and Architecture=arm64.

Remaining issue:
  The raw non-editor game binary cannot load uncooked UE assets and fails while loading /Engine/EngineMaterials/WorldGridMaterial.

Conclusion:
  The runtime target is buildable, but a successful editor/cooker/package path is still required.
```

GX10 editor/cooker blocker:

```text
BlocksEditor Linux Development is blocked by Autodesk FBX SDK:
  FBX SDK not found in ../Binaries/ThirdParty/FBX/2020.2/Linux/aarch64-unknown-linux-gnueabi/

Epic UE5.5 gitdeps includes:
  Engine/Binaries/ThirdParty/FBX/2020.2/Linux/x86_64-unknown-linux-gnu/libfbxsdk.so

Epic UE5.5 gitdeps does not include:
  Engine/Binaries/ThirdParty/FBX/2020.2/Linux/aarch64-unknown-linux-gnueabi/libfbxsdk.so
```

FBX policy for this migration:

```text
Do not disable FBX just to make the build shorter.
First choice is a real LinuxArm64 FBX SDK 2020.2 binary compatible with UE5.5.
If no real LinuxArm64 FBX SDK binary can be obtained, FBX import/export is a verified no-native-dependency case and may be compiled out for the GX10 editor/cooker build only.
Do not use an x86_64 libfbxsdk.so on ARM64; it cannot link into the LinuxArm64 editor.
Do not use a fake/stub libfbxsdk.so as a success criterion; that would hide linker/runtime failures and would not make FBX functional.
```

Impact of compiling out FBX on GX10, if required:

```text
Expected acceptable impact:
  GX10 editor/cooker loses FBX import/export features.
  Already-imported FSDS assets can still be cooked if their serialized .uasset data is valid after UE5 conversion.

Unacceptable impact:
  Any change that disables ChaosVehiclesPlugin, AirSim, project maps, sensors, RPC port 41451, or ROS /fsds topic compatibility.
```

Next required native-GX10 steps:

1. Decide whether a real LinuxArm64 Autodesk FBX SDK 2020.2 binary can be supplied.
2. If yes, stage it at:

```text
/home/hard/UnrealEngine-5.5/Engine/Binaries/ThirdParty/FBX/2020.2/Linux/aarch64-unknown-linux-gnueabi/libfbxsdk.so
```

3. If no, compile out FBX-dependent GX10 editor/cooker paths and document each disabled module/feature.
4. Re-run:

```bash
UE_USE_SYSTEM_DOTNET=1 \
UE_ISPC_PATH=/tmp/ue-ispc-arm64/usr/bin/ispc \
LD_LIBRARY_PATH=/tmp/ue-ispc-arm64/usr/lib/aarch64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH} \
/home/hard/UnrealEngine-5.5/Engine/Build/BatchFiles/Linux/Build.sh BlocksEditor Linux Development \
  -Project=/home/hard/Desktop/Formula-Student-Driverless-Simulator/UE4Project/FSOnline.uproject \
  -ForceUseSystemCompiler -buildubt
```

5. After the editor/cooker builds, open or commandlet-convert the UE5 project assets and fix Blueprint migration errors.
6. Package a LinuxArm64 simulator and verify TCP `41451`.
7. Run the ROS 2 Jazzy bridge acceptance test:

```bash
source /opt/ros/jazzy/setup.bash
source ros2/install/setup.bash
ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py host:=127.0.0.1 timeout:=120.0
```

Reference notes:

```text
Autodesk FBX SDK 2020 Linux docs list gcc x86 and gcc x64 runtime libraries.
Autodesk FBX SDK 2020.2 notes mention Apple M1 universal binaries, not Linux ARM64.
```
