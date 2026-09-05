# DIGIT Tactile Sensor — ROS2 Integration Toolkit

A complete, working pipeline that takes a [DIGIT](https://digit.ml/) tactile
sensor — a small, low-cost, open-source vision-based fingertip sensor
originally developed by Meta/FAIR — all the way from "plugged into a laptop"
to a ROS2 system that detects contact, estimates grip force, reconstructs 3D
contact shape, and makes gripper decisions (close / hold / detect slip /
regrasp) in real time. It also includes a full robot-free simulation path, so
you can demonstrate the entire sensing→decision→actuation loop without owning
a robot.

**This guide assumes zero prior experience with the DIGIT sensor.** If you've
never touched one before, start at the top and work down — every step
explains not just *what* to run, but *why*.

---

## Table of contents

1. [What is DIGIT, and what does this repo actually do?](#1-what-is-digit-and-what-does-this-repo-actually-do)
2. [Architecture overview](#2-architecture-overview)
3. [Repository structure](#3-repository-structure)
4. [Prerequisites](#4-prerequisites)
5. [Step-by-step setup](#5-step-by-step-setup)
6. [Running the full pipeline](#6-running-the-full-pipeline)
7. [Optional: calibrated 3D depth reconstruction](#7-optional-calibrated-3d-depth-reconstruction)

---

## 1. What is DIGIT, and what does this repo actually do?

DIGIT is a small tactile sensor shaped like a fingertip. Physically, it's
just a tiny USB camera pointed at the underside of a soft, transparent gel
pad, illuminated from several directions by colored LEDs. When something
presses into the gel, the camera sees the gel deform, and — because the
lighting comes from multiple known directions — the *color* of the
deformation encodes information about its 3D shape (a technique called
**photometric stereo**). To your computer, DIGIT shows up as nothing more
exotic than a plain USB webcam.

That simplicity is deceptive. Getting genuinely *useful* signals out of that
raw video feed — contact detection, grip-force estimation, real 3D depth,
and decisions a robot controller could act on — requires a real software
pipeline. This repository **is** that pipeline:

- Raw video → **contact detection** (is something touching the gel, where, how big)
- Raw video → **pressure/force proxy** (how hard, is it holding steady or slipping)
- Raw video → **calibrated 3D depth** (actual millimeter-scale contact geometry)
- All of the above → **a decision-making node** that behaves like a real
  gripper controller (close → hold → detect slip → regrasp)
- A **simulation mode** using [TACTO](https://github.com/facebookresearch/tacto)
  that drives a simulated robot gripper from the exact same decision logic —
  so you can demonstrate the whole loop with no physical robot at all

Everything is built as a modular [ROS2](https://docs.ros.org/) (Humble)
package, so it's ready to plug into an actual robot's control stack later.

---

## 2. Architecture overview

The single biggest surprise in this project: **DIGIT's raw video feed does
not survive USB virtualization.** If you try to pass the sensor's USB
connection into a VM (VMware, VirtualBox) or into WSL2 via `usbipd`, the
video comes out corrupted (banded rainbow artifacts) and/or severely
throttled (a few frames per second instead of 30). This is a documented,
structural limitation of how isochronous USB transfers (the kind cameras
use) survive virtualization on Windows.

The fix this repo uses: **never pass the raw USB connection into Linux at
all.** Instead:

```
┌─────────────────────────────┐         ┌──────────────────────────────────────┐
│  WINDOWS (native, no VM)     │         │  LINUX (WSL2, VM, or robot computer)  │
│                              │         │                                        │
│  DIGIT sensor (USB)          │         │   ROS2 (digit_ros2 package)           │
│       │                      │         │   ┌────────────────────────────┐     │
│       ▼                      │         │   │ camera_publisher_node       │     │
│  windows_stream_server.py    │  JPEG   │   │  /digit/image_raw           │     │
│  (reads camera natively,     │ over TCP│   └──────────┬─────────────────┘     │
│   encodes JPEG, serves it    ├────────►│              │                        │
│   over a plain TCP socket)   │         │   ┌──────────▼────────┐  ┌──────────┐│
│                              │         │   │ contact_detector    │  │pressure  ││
└─────────────────────────────┘         │   │ /digit/contact       │  │estimator ││
                                          │   └──────────┬────────┘  └────┬─────┘│
   ...or, with no physical sensor:       │              │                 │      │
                                          │   ┌──────────▼─────────────────▼───┐ │
┌─────────────────────────────┐         │   │      grasp_decision              │ │
│  TACTO + PyBullet simulation │  same   │   │  /digit/gripper_command          │ │
│  (simulated DIGIT + gripper) ├────────►│   │  (OPEN/CLOSE/HOLD/REGRASP)       │ │
│  tacto_stream_bridge.py      │protocol │   └──────────┬────────────────────────┘ │
└─────────────────────────────┘         │              │                        │
                                          │   ┌──────────▼────────┐              │
                                          │   │ depth_reconstructor │ (optional)  │
                                          │   │ /digit/depth        │             │
                                          │   └────────────────────┘             │
                                          └──────────────────────────────────────┘
```

Two things fall out of this design that are worth understanding up front:

- **The exact same ROS2 pipeline runs unmodified against real hardware or
  simulation.** Only the *source* of `/digit/image_raw` changes (real camera
  feed over TCP vs. simulated TACTO render over the identical TCP protocol).
  Nothing downstream knows or cares which one it's getting.
- **A plain network socket, not USB, crosses the Windows↔Linux boundary.**
  Regular TCP traffic has none of the real-time isochronous scheduling
  requirements that break under virtualization, so this sidesteps the whole
  problem rather than trying to patch around it.

---

## 3. Repository structure

```
digit-sensor/              Windows-side scripts (plain Python + OpenCV, no ROS2)
├── contact_detection.py       Live contact detection (reference-frame diff)
├── contact_heatmap.py         Qualitative deformation-intensity heatmap
├── pressure_estimate.py       Uncalibrated pressure/force proxy with live graph
├── calibrate_capture.py       Depth-calibration data collector (needs a small ball)
├── build_lookup_table.py      Turns calibration samples into a depth lookup table
├── reconstruct_depth.py       Standalone calibrated 3D depth reconstruction
└── windows_stream_server.py   The network bridge - streams the sensor to Linux

digit_ws/                  The ROS2 workspace (colcon)
└── src/
    ├── digit_interfaces/          Custom message types (ContactInfo, PressureEstimate)
    └── digit_ros2/                 Main package
        ├── digit_ros2/
        │   ├── camera_publisher_node.py     Ingests either a local camera or a network stream
        │   ├── contact_detector_node.py     Publishes /digit/contact
        │   ├── pressure_estimator_node.py   Publishes /digit/pressure
        │   ├── depth_reconstructor_node.py  Publishes /digit/depth (needs calibration data)
        │   ├── grasp_decision_node.py       The gripper-controller state machine
        │   └── sim_gripper_forwarder_node.py Forwards gripper commands into the TACTO sim
        └── launch/
            ├── digit.launch.py         Shared base launch file (all parameters)
            ├── digit_wsl2.launch.py    Preset: network-streaming mode (dev machine)
            └── digit_robot.launch.py   Preset: local camera mode (sensor plugged directly in)

simulation/                 Robot-free simulation bridge
├── tacto_stream_bridge.py     Runs a simulated DIGIT + gripper, bridges it into ROS2
└── README.md                  Setup instructions specific to the simulation
```

---

## 4. Prerequisites

- **A DIGIT sensor**, plugged into a PC via USB.
- **Windows 10/11** with Python 3.9+ installed.
- **A Linux environment for ROS2**: WSL2 running Ubuntu 22.04 is what this
  guide uses (recommended for anyone without a spare Linux machine), but a
  real Ubuntu 22.04 install or the eventual robot's own onboard computer
  works too.
- **ROS2 Humble** installed in that Linux environment.

---

## 5. Step-by-step setup

### Step 5.1 — Find your DIGIT's camera index (Windows)

Windows assigns cameras arbitrary index numbers. Find yours:

```powershell
python -m venv digit_env
.\digit_env\Scripts\Activate.ps1
pip install opencv-python
```

```python
import cv2
for i in range(5):
    cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
    if not cap.isOpened():
        continue
    ok, frame = cap.read()
    if ok:
        print(f"Index {i}: frame shape {frame.shape}")
        cv2.imshow(f"Camera index {i}", frame)
        cv2.waitKey(1500)
        cv2.destroyAllWindows()
    cap.release()
```

Whichever index shows the colorful DIGIT gel image (soft, blurred color
blobs — that's the sensor's internal LED illumination, not a broken camera)
is the one you'll use below with `--device-index`.

### Step 5.2 — Try the sensor natively first

Before anything involving ROS2 or networking, confirm the sensor itself
works:

```bash
pip install scipy
python digit-sensor/contact_detection.py --device-index 1
```

(replace `1` with your index from step 5.1). Press **`r`** with nothing
touching the gel to set a reference frame, then press on it — you should see
a green contour tracking the contact area live. If this works, your sensor
and drivers are fine and any later problems are software/networking, not
hardware.

### Step 5.3 — Set up WSL2 + Ubuntu + ROS2 Humble

In an administrator PowerShell:

```powershell
wsl --install -d Ubuntu-22.04
```

Reboot if prompted, then open "Ubuntu" from the Start menu once to finish
first-time setup (pick a Linux username/password). Inside that Ubuntu
terminal, install ROS2 Humble:

```bash
sudo apt update
sudo apt install software-properties-common curl -y
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions ros-humble-cv-bridge python3-opencv python3-scipy -y
```

**Important:** install OpenCV and NumPy via `apt` here, never `pip`. See
[the FAQ](#9-troubleshooting--faq) for why — it's the single most common
thing that breaks this stack.

### Step 5.4 — Clone this repo and build the ROS2 workspace

```bash
git clone https://github.com/landradepro/digit-tactile-sensor.git ~/digit_ws
cd ~/digit_ws
python3 -m venv digit_env
source digit_env/bin/activate
colcon build
source install/setup.bash
```

`--system-site-packages` is required here — it lets this venv see the
`apt`-installed `rclpy`/`cv_bridge`/`opencv` while still keeping any `pip`
installs isolated to the venv.

### Step 5.5 — Start the Windows-side streaming bridge

Back on Windows:

```powershell
python digit-sensor/windows_stream_server.py --device-index 1 --port 8090
```

Leave this running — it's what makes the sensor visible to Linux.

### Step 5.6 — Find your Windows PC's IP address

```powershell
ipconfig
```

Note the **IPv4 Address** of your active network adapter. WSL2 can reach the
Windows host over the network directly using this address.

Set it once so you don't have to retype it every time:

```bash
echo 'export DIGIT_STREAM_HOST=<your-ip>' >> ~/.bashrc
source ~/.bashrc
```

---

## 6. Running the full pipeline

With `windows_stream_server.py` running on Windows and `DIGIT_STREAM_HOST`
set in WSL2:

```bash
cd ~/digit_ws
source digit_env/bin/activate
source install/setup.bash
ros2 launch digit_ros2 digit_wsl2.launch.py
```

This starts every node: the camera publisher, contact detector, pressure
estimator, and grasp decision state machine (depth reconstruction is off by
default until you've done the calibration in [section 7](#7-optional-calibrated-3d-depth-reconstruction)).

**Two of the nodes need a one-time "zero point" before they report anything
meaningful** — they work by comparing each frame against a reference frame
captured with nothing touching the gel:

```bash
ros2 service call /contact_detector/capture_reference std_srvs/srv/Trigger
ros2 service call /pressure_estimator/capture_reference std_srvs/srv/Trigger
```

Now watch it react to a real touch:

```bash
ros2 topic echo /digit/contact              # touching, area, center
ros2 topic echo /digit/pressure             # pressure_proxy - a force proxy
ros2 topic echo /digit/gripper_command      # OPEN / CLOSE / HOLD / REGRASP
```

Press the gel gradually and you should see `OPEN → CLOSE → HOLD`; slide or
partially lift while held to see `SLIP_DETECTED → REGRASP`. The exact
pressure numbers vary sensor-to-sensor and lighting-to-lighting, so if you
never reach `HOLD`, check your actual readings and retune:

```bash
ros2 topic echo /digit/pressure                                # find your real peak value
ros2 launch digit_ros2 digit_wsl2.launch.py target_pressure:=<value below your peak>
```

**Deploying on a real robot instead?** Once DIGIT is plugged directly into
the robot's own Ubuntu computer (no network streaming needed — no
virtualization in the way there), use the other preset:

```bash
ros2 launch digit_ros2 digit_robot.launch.py device_index:=0
```

---

## 7. Optional: calibrated 3D depth reconstruction

Contact detection and pressure are enough for most closed-loop grasp logic,
but DIGIT's real party trick is reconstructing actual 3D contact geometry in
millimeters. This needs a one-time calibration using a small rigid sphere of
known diameter (a steel BB, a bearing ball).

```bash
python digit-sensor/calibrate_capture.py --device-index 1
```

Follow the on-screen instructions: set a reference frame, calibrate the
pixel-to-mm scale by moving the ball a known distance along a ruler, then
press the ball onto the gel at 30-50 different spots and force levels.

```bash
python digit-sensor/build_lookup_table.py
python digit-sensor/reconstruct_depth.py --device-index 1
```

The second command opens a live view with a colorized depth map and a
"peak depth in mm" readout — check that it looks physically plausible (a
firm fingertip press should read as a few mm, not 50mm or 0.01mm).

To use this inside ROS2, copy the resulting `digit-sensor/calibration_data/`
folder to the Linux side (default expected path
`~/digit_ws/calibration_data`), then launch with depth enabled:

```bash
ros2 launch digit_ros2 digit_wsl2.launch.py enable_depth:=true
```
