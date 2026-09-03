# DIGIT Tactile Sensor — ROS2 Integration Toolkit

A complete, working pipeline that takes a [DIGIT](https://digit.ml/) tactile
sensor — a small, low-cost, open-source vision-based fingertip sensor
originally developed by Meta/FAIR — and turns its raw video feed into a
ROS2 system that detects contact, estimates grip force, reconstructs 3D
contact shape, and makes gripper decisions (close / hold / detect slip /
regrasp) in real time. It also includes a full robot-free simulation path
using [TACTO](https://github.com/facebookresearch/tacto), so you can
demonstrate the entire sensing→decision→actuation loop without owning a
robot at all.

**This guide assumes zero prior experience with the DIGIT sensor.** If
you've never touched one before, start at the top and work down — every
step explains not just *what* to run, but *why*.

> **Is your DIGIT sensor plugged into a machine that can't run ROS2
> directly** — a Windows PC, or a Linux VM/WSL2 instance without working USB
> passthrough to the sensor? That's a real, documented limitation (USB
> isochronous video doesn't survive virtualization) with its own dedicated
> workaround. See the **[`windows-streaming-dev`](../../tree/windows-streaming-dev)
> branch** for a network-streaming bridge that solves it. This branch
> (`main`) assumes DIGIT is plugged directly into the Linux machine running
> ROS2.

---

## Table of contents

1. [What is DIGIT, and what does this repo actually do?](#1-what-is-digit-and-what-does-this-repo-actually-do)
2. [Architecture overview](#2-architecture-overview)
3. [Repository structure](#3-repository-structure)
4. [Prerequisites](#4-prerequisites)
5. [Step-by-step setup](#5-step-by-step-setup)
6. [Running the full pipeline](#6-running-the-full-pipeline)
7. [Optional: calibrated 3D depth reconstruction](#7-optional-calibrated-3d-depth-reconstruction)
8. [No physical robot? Simulate one with TACTO](#8-no-physical-robot-simulate-one-with-tacto)
9. [Troubleshooting / FAQ](#9-troubleshooting--faq)
10. [Technical challenges this project solved](#10-technical-challenges-this-project-solved)
11. [Roadmap](#11-roadmap)

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

That simplicity is deceptive. Getting genuinely *useful* signals out of
that raw video feed — contact detection, grip-force estimation, real 3D
depth, and decisions a robot controller could act on — requires a real
software pipeline. This repository **is** that pipeline:

- Raw video → **contact detection** (is something touching the gel, where, how big)
- Raw video → **pressure/force proxy** (how hard, is it holding steady or slipping)
- Raw video → **calibrated 3D depth** (actual millimeter-scale contact geometry)
- All of the above → **a decision-making node** that behaves like a real
  gripper controller (close → hold → detect slip → regrasp)
- A **simulation mode** using TACTO that drives a simulated robot gripper
  from the exact same decision logic — so you can demonstrate the whole
  loop with no physical robot at all

Everything is built as a modular [ROS2](https://docs.ros.org/) (Humble)
package, ready to plug into an actual robot's control stack.

---

## 2. Architecture overview

DIGIT is just a UVC (standard) camera as far as the OS is concerned, so
when it's plugged directly into the Linux machine running ROS2, the setup
is refreshingly simple:

```
┌───────────────────────────────────────────────────────────────────────┐
│  LINUX (native - DIGIT plugged in directly, no virtualization at all) │
│                                                                         │
│   DIGIT sensor (USB)                                                   │
│        │                                                                │
│        ▼                                                                │
│   ┌────────────────────────────┐                                       │
│   │ camera_publisher_node       │  publishes /digit/image_raw          │
│   └──────────┬──────────────────┘                                      │
│              │                                                          │
│   ┌──────────▼────────┐  ┌────────────────┐  ┌────────────────────┐   │
│   │ contact_detector    │  │pressure         │  │ depth_reconstructor │  │
│   │ /digit/contact       │  │estimator        │  │ /digit/depth         │  │
│   │                      │  │/digit/pressure  │  │ (optional, needs      │  │
│   └──────────┬────────┘  └────┬───────────┘  │  calibration data)   │   │
│              │                 │               └──────────────────────┘  │
│   ┌──────────▼─────────────────▼───┐                                    │
│   │      grasp_decision              │  publishes /digit/gripper_command │
│   │  (close / hold / slip / regrasp) │  (OPEN / CLOSE / HOLD / REGRASP)  │
│   └──────────────────────────────────┘                                  │
└───────────────────────────────────────────────────────────────────────┘
```

No physical sensor yet, or want to demonstrate closed-loop grasp control
without a robot? [Section 8](#8-no-physical-robot-simulate-one-with-tacto)
swaps the top of this diagram for a PyBullet+TACTO simulation that feeds
the **exact same pipeline** below it — nothing downstream changes.

---

## 3. Repository structure

```
digit_ws/                       The ROS2 workspace (colcon)
└── src/
    ├── digit_interfaces/           Custom message types (ContactInfo, PressureEstimate)
    └── digit_ros2/                  Main package
        ├── digit_ros2/
        │   ├── camera_publisher_node.py     Publishes /digit/image_raw (local camera or network stream)
        │   ├── contact_detector_node.py     Publishes /digit/contact
        │   ├── pressure_estimator_node.py   Publishes /digit/pressure
        │   ├── depth_reconstructor_node.py  Publishes /digit/depth (needs calibration data)
        │   ├── grasp_decision_node.py       The gripper-controller state machine
        │   └── sim_gripper_forwarder_node.py Forwards gripper commands into the TACTO sim
        └── launch/
            ├── digit.launch.py         Shared base launch file (all parameters)
            ├── digit_robot.launch.py   Preset: sensor plugged directly into this machine
            └── digit_sim.launch.py     Preset: TACTO simulation (no hardware needed)

simulation/                      Robot-free simulation bridge
├── tacto_stream_bridge.py           Runs a simulated DIGIT + gripper, bridges it into ROS2
└── README.md                        Setup instructions specific to the simulation
```

---

## 4. Prerequisites

- **A DIGIT sensor**, plugged directly into the same machine that will run
  ROS2 (or skip this and go straight to the [TACTO simulation](#8-no-physical-robot-simulate-one-with-tacto)
  if you don't have one yet).
- **Ubuntu 22.04** with **ROS2 Humble** installed.
- Basic comfort with a terminal. No prior ROS2 or computer-vision
  experience is assumed beyond that.

---

## 5. Step-by-step setup

### Step 5.1 — Install ROS2 Humble (skip if already installed)

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

### Step 5.2 — Find your DIGIT's camera index

```bash
v4l2-ctl --list-devices
```

(install with `sudo apt install v4l-utils` if missing). DIGIT will show up
as a UVC camera device, e.g. `/dev/video2` — the number at the end (`2`) is
the index you'll use below.

### Step 5.3 — Clone this repo and build the workspace

```bash
git clone https://github.com/landradepro/digit-tactile-sensor.git ~/digit_ws
cd ~/digit_ws
python3 -m venv --system-site-packages digit_env
source digit_env/bin/activate
colcon build
source install/setup.bash
```

`--system-site-packages` is required — it lets this venv see the
`apt`-installed `rclpy`/`cv_bridge`/`opencv` while still keeping any `pip`
installs isolated to the venv.

---

## 6. Running the full pipeline

```bash
cd ~/digit_ws
source digit_env/bin/activate
source install/setup.bash
ros2 launch digit_ros2 digit_robot.launch.py device_index:=2
```

(replace `2` with your index from step 5.2). This starts every node: the
camera publisher, contact detector, pressure estimator, and grasp decision
state machine (depth reconstruction is on by default with this preset —
see [section 7](#7-optional-calibrated-3d-depth-reconstruction) for the
one-time calibration it needs, or pass `enable_depth:=false` to skip it).

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
ros2 launch digit_ros2 digit_robot.launch.py device_index:=2 target_pressure:=<value below your peak>
```

---

## 7. Optional: calibrated 3D depth reconstruction

Contact detection and pressure are enough for most closed-loop grasp logic,
but DIGIT's real party trick is reconstructing actual 3D contact geometry
in millimeters. This needs a one-time calibration using a small rigid
sphere of known diameter (a steel BB, a bearing ball, or a 3D-printed ball
work fine), producing a `calibration_data/` folder that
`depth_reconstructor_node` reads (default expected path
`~/digit_ws/calibration_data`, overridable via the `calibration_dir`
parameter).

The calibration tooling itself lives on the
**[`windows-streaming-dev`](../../tree/windows-streaming-dev)** branch
(`digit-sensor/calibrate_capture.py`, `build_lookup_table.py`,
`reconstruct_depth.py`) since it was originally built alongside that
workflow — check that branch's README for the full calibration walkthrough,
then bring the resulting `calibration_data/` folder back here and launch
with:

```bash
ros2 launch digit_ros2 digit_robot.launch.py device_index:=2 enable_depth:=true
```

---

## 8. No physical robot? Simulate one with TACTO

This repo includes a full simulation path using
[TACTO](https://github.com/facebookresearch/tacto), a PyBullet-based
simulator built specifically for GelSight/DIGIT-style tactile sensors. It
simulates a parallel-jaw gripper with two DIGIT sensors (one per finger)
grasping an object, and connects to the **exact same** ROS2 pipeline used
for real hardware.

Full setup instructions are in [`simulation/README.md`](simulation/README.md)
(it needs its own isolated Python environment — TACTO's dependency chain is
old and unmaintained, and keeping it separate avoids any risk to the ROS2
setup). Once its bridge script is running:

```bash
ros2 launch digit_ros2 digit_sim.launch.py
```

You'll see a real gripper close, hold, and react to a simulated slip event
— driven entirely by your live sensor-processing pipeline, with no
physical hardware anywhere in the loop.

---

## 9. Troubleshooting / FAQ

**`ros2 topic hz` shows a low, jittery rate even though everything looks
connected.**
Check whether your ROS2 node has a background `threading.Thread` doing I/O
alongside `rclpy.spin()`. Python's GIL can starve the ROS2 executor when a
separate thread is busy, tanking the actual publish rate even though the
callback code itself looks trivial. Fix: do the I/O directly inside the
timer callback (non-blocking, polled), not in a separate thread.

**`cv_bridge` throws `KeyError` on `cv2_to_imgmsg`, or an `AttributeError`
about `_ARRAY_API not found` mentioning NumPy.**
Something installed a `pip` version of OpenCV or NumPy that doesn't match
what `cv_bridge` (installed via `apt`) was compiled against — usually a
stray `pip install opencv-python`. Check what's actually being imported:

```bash
python3 -c "import cv2; print(cv2.__file__, cv2.__version__)"
python3 -c "import numpy; print(numpy.__file__, numpy.__version__)"
```

`cv2.__file__` should point to `/usr/lib/python3/dist-packages/...` at
version `4.5.4` (Ubuntu 22.04's apt version), and `numpy` should be a `1.x`
version — not something under `~/.local` or `/usr/local`. If it isn't:

```bash
pip uninstall opencv-python opencv-python-headless numpy -y
sudo apt install --reinstall python3-opencv
```

**I'm running this in a VM or WSL2 and the video is corrupted (rainbow
banding) or stuck at a few frames per second.**
This is a real, documented limitation: isochronous USB video (what cameras
use) doesn't survive virtualization/passthrough on Windows, and no config
tweak fixes it. See the [`windows-streaming-dev`](../../tree/windows-streaming-dev)
branch for the network-streaming workaround this project built for exactly
that situation.

**`ros2 param set` doesn't seem to change anything.**
If a node reads a parameter once in `__init__` and caches it in an instance
variable, `ros2 param set` updates the parameter server but never touches
that cached value. Nodes in this repo that need live tuning (like
`grasp_decision`) re-read `self.get_parameter(...).value` fresh inside the
callback specifically to avoid this.

**A launch argument I pass doesn't seem to do anything.**
`ros2 launch` silently ignores arguments a launch file doesn't declare —
it won't error. Double check the argument name matches exactly what that
specific launch file (`digit.launch.py` vs `digit_robot.launch.py` vs
`digit_sim.launch.py`) actually declares.

---

## 10. Technical challenges this project solved

Worth knowing even if you never hit these yourself — each one shaped a real
design decision in this repo, not just a one-off fix:

- **USB isochronous video does not survive virtualization.** Discovered
  while developing against a sensor connected to a Windows machine: tried
  and ruled out VMware `.vmx` passthrough tuning, USB controller version
  switching, resolution reduction, and WSL2's `usbipd-win` USB/IP
  redirection (a documented structural limitation, not a config issue).
  Solved on the `windows-streaming-dev` branch by never crossing that
  boundary with raw USB at all — streaming already-decoded JPEG frames over
  a plain TCP socket instead. This `main` branch sidesteps the whole
  problem simply by assuming direct hardware access.
- **A background thread inside a ROS2 node can silently wreck its publish
  rate.** An early version of the network-streaming camera node (see the
  other branch) used a daemon thread for socket I/O; the actual topic rate
  was ~2Hz against a 30Hz timer, due to GIL contention with the `rclpy`
  executor. Fixed by polling I/O directly inside the timer callback instead
  — a lesson that applies to any ROS2 node design, not just that one.
- **`cv_bridge` and Python's OpenCV/NumPy ecosystem are ABI-fragile
  together.** `apt`'s `cv_bridge` is compiled against a specific
  OpenCV/NumPy build; a stray `pip install opencv-python` (which pulls a
  much newer, incompatible OpenCV/NumPy) breaks it with a non-obvious
  `KeyError`. This recurred in two independently-built fresh environments,
  which is why it's called out explicitly in the FAQ above rather than
  treated as a one-off.
- **A cached-at-init parameter silently ignored later `ros2 param set`
  calls.** `grasp_decision`'s pressure threshold was read once into an
  instance variable at startup; `ros2 param set` updated the parameter
  server correctly but the node never re-read it, so nothing tuned via CLI
  ever took effect. Fixed by reading parameters fresh inside the callback.
- **Archived research code needs archaeology, not despair.** Getting TACTO
  running required fixing four independent instances of the same root
  cause in its dependency chain — old, unmaintained packages (`attrdict`,
  `networkx`, `urdfpy`) using Python/NumPy APIs that were later removed —
  one at a time, in an isolated environment so none of it could risk the
  working ROS2 setup.

---

