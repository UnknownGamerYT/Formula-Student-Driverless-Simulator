# GX10 Remote PC Requirements

This file is for users connecting to the GX10 from their own Windows, macOS, or Linux PC.

The normal workflow uses noVNC in a browser, so you do not need to install a native VNC viewer.

## What You Need

- Network access to the GX10, usually through Tailscale or the lab network
- The GX10 SSH target: `hard@100.81.202.64`
- The Linux password for user `hard`
- A terminal with `ssh`
- A modern browser
- Optional: Foxglove Studio if you want to inspect ROS topics visually
- Optional: a browser tab for TensorBoard if RL training is running

The VNC password is the same as the PC/Linux password for user `hard`.

You do not need to install ROS, CUDA, PyTorch, YOLO, Unreal Engine, or the simulator on your own PC for the normal remote workflow. Those run on the GX10.

## Windows PC

Use Windows Terminal, PowerShell, or Command Prompt.

Check that OpenSSH is installed:

```powershell
ssh -V
```

If `ssh` is not found, install the Windows OpenSSH client from an Administrator PowerShell:

```powershell
Add-WindowsCapability -Online -Name OpenSSH.Client~~~~0.0.1.0
```

Use one of these browsers:

- Microsoft Edge
- Chrome
- Firefox

You do not need TigerVNC, RealVNC, or TightVNC for the normal browser workflow.

## macOS PC

OpenSSH is included with macOS.

Check it:

```bash
ssh -V
```

Use Terminal, iTerm2, or another terminal app.

Use one of these browsers:

- Safari
- Chrome
- Firefox
- Edge

You do not need macOS Screen Sharing for the normal browser workflow.

## Linux PC

Install an SSH client if it is missing.

Ubuntu/Debian:

```bash
sudo apt update
sudo apt install -y openssh-client
```

Fedora:

```bash
sudo dnf install -y openssh-clients
```

Arch:

```bash
sudo pacman -S openssh
```

Use one of these browsers:

- Firefox
- Chrome
- Chromium
- Edge

You do not need a native VNC viewer for the normal browser workflow.

## Quick Connection Test

From your PC:

```bash
ssh hard@100.81.202.64
```

If login succeeds, your PC is ready for the daily runbook.

If login fails:

- Confirm you are on Tailscale or the correct lab network.
- Confirm the GX10 IP is still `100.81.202.64`.
- Confirm you are using the `hard` account password.
- Ask the team whether the GX10 is powered on and connected.

## Browser URL Used During The Runbook

After the runbook starts the noVNC services and opens the SSH tunnel, use this URL from your PC browser:

```text
http://localhost:6080/vnc.html?host=localhost&port=6080
```

The `localhost` in this URL means your own PC. The SSH tunnel forwards your browser to the GX10.

## Expected Long-Running SSH Tunnel

This command is used in the runbook:

```bash
ssh -N -L 6080:localhost:6080 hard@100.81.202.64
```

After you enter the password, it intentionally appears to hang. That is normal. Leave it open while using the browser desktop.

## Optional Foxglove

If you want Foxglove Studio on your PC, install it from the official Foxglove website or use your team-approved installation method.

The runbook tunnels Foxglove with:

```bash
ssh -N -L 8765:localhost:8765 hard@100.81.202.64
```

Then connect Foxglove to:

```text
ws://localhost:8765
```

For the autonomy stack, useful Foxglove panels are:

- 3D panel for `/autonomy/viz/map_markers` and the path topics
- Image panel for `/autonomy/viz/camera/cam1_overlay`
- Raw Messages panel for `/autonomy/race_state`
- Raw Messages panel for `/autonomy/offtrack_reset_status`

## Optional TensorBoard

If RL training is running on the GX10, the runbook tunnels TensorBoard with:

```bash
ssh -N -L 6006:localhost:6006 hard@100.81.202.64
```

Then open:

```text
http://localhost:6006
```
