# marc-sdk

**MARC 2026 (MetaSejong AI Robot Challenge) participant SDK**

`MARCClient` handles the ROS 2 communication with the platform for you — the registration
handshake, distinguishing message types, session and sequence management, QoS, and CCTV
camera auto-discovery. You only register event handlers with decorators and call the
submit/control/sensor methods, so you can **focus on perception (VLA) and driving logic**.

- Protocol specification and SDK/API reference (single source of truth): the **API Reference**
  and **Technical Guide** pages of the developer guide — https://marc-challenge.github.io/marc-dev-guide/

## Requirements

- Ubuntu 22.04 + **ROS 2 Humble** (`/opt/ros/humble`), Python 3.10
- `rclpy`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs` - provided by ROS 2 Humble.
  This package has no hard PyPI dependencies, and **must be used after sourcing the ROS 2 environment**.

## Getting Started

`marc_sdk` is not a pip-installed package; it is **distributed as source**. You only need to
add its source path to `PYTHONPATH` so Python can find it (the submission Docker image sets
this up automatically, see demo/Dockerfile and demo/launch.sh).

```bash
source /opt/ros/humble/setup.bash
# Add the parent folder of marc_sdk (= this repo root) to PYTHONPATH
export PYTHONPATH="$(pwd):${PYTHONPATH}"
```

## 60-Second Quickstart

```python
from marc_sdk import MARCClient

client = MARCClient.from_env()      # MARC_TEAM_ID / MARC_TOKEN env vars

@client.on_mission                  # Stage 1 (msg 201)
def handle(mission):
    # mission.voice_command, mission.round, ...
    result = my_vla.process(mission)        # <- participant algorithm
    client.submit_grounding(result)         # msg 301

@client.on_stage2_mission           # Stage 2 interpretation (msg 211)
def handle2(mission):
    client.submit_stage2_grounding(my_vla.process(mission))   # msg 311

@client.on_stage2_run               # separate thread on entering STAGE2_RUN
def drive():
    while client.is_running:
        pose = client.get_world_pose()
        client.send_cmd_vel(linear_x=0.3)   # <- participant navigation
    client.task_complete()                  # msg 302

client.connect()                    # register -> ack
client.run()                        # spin (exit with Ctrl-C)
```

For a complete runnable example, see [`demo/participant_app.py`](demo/participant_app.py).

## Core API Summary

| Category | Method / Decorator |
|------|-------------------|
| Connection | `MARCClient.from_env()`, `connect()`, `run()`, `shutdown()` |
| Stage 1 | `@on_mission`, `submit_grounding(...)` |
| Stage 2 | `@on_stage2_mission`, `submit_stage2_grounding(...)`, `@on_stage2_run`, `task_complete()` |
| Control | `send_cmd_vel(...)`, `stop()`, `send_arm_command(...)` |
| Sensors | `list_cctv()`, `get_cctv_image(id)`, `get_robot_image(which)`, `get_odom()`, `get_lidar()`, `get_occupancy_map()`, `get_world_pose()` |
| Notifications | `@on_state_change`, `@on_time_remaining`, `@on_time_expired`, `@on_score`, `@on_transition`, `@on_warning` |

> Sensor getters return the **raw ROS 2 messages** (`sensor_msgs/Image`, etc.) as-is. numpy/OpenCV
> conversion is left to the participant, using whichever library they prefer.

## License

MIT
