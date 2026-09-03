# TACTO simulation bridge

`tacto_stream_bridge.py` runs a PyBullet + TACTO simulation of a parallel-jaw
gripper with two simulated DIGIT sensors, one per finger. It streams the
simulated tactile image over the same TCP protocol the real hardware bridge
(`digit-sensor/windows_stream_server.py`) uses, and listens for gripper
open/close/hold/regrasp commands over a second socket - allowing the exact
same ROS2 pipeline (`contact_detector`, `pressure_estimator`,
`grasp_decision`) to run unmodified against either real hardware or this
simulation.

## Setup

Requires a separate, isolated Python environment (kept apart from the ROS2
`digit_env` venv to avoid dependency conflicts - TACTO's dependency chain
pulls in old, unmaintained packages that need specific NumPy pins):

```bash
python3 -m venv ~/tacto_env
source ~/tacto_env/bin/activate

git clone https://github.com/facebookresearch/tacto.git ~/tacto_repo
cd ~/tacto_repo
pip install -e .
pip install -r requirements/examples.txt

# Known fixes needed for this (archived, unmaintained) dependency chain on
# modern Python/NumPy:
pip uninstall attrdict -y
pip install attrdict3
pip install --upgrade networkx
pip install "numpy<1.24"

cp ~/digit_ws/simulation/tacto_stream_bridge.py ~/tacto_repo/examples/

