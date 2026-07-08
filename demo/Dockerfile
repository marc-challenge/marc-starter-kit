# Copyright (c) 2026, IoT Convergence & Open Sharing System (IoTCOSS)
#
# MARC 2026 - Participant demo application (Docker submission template)
# ---------------------------------------------------------------------------
# The participant application must be developed/submitted as a Docker image (required).
# The standard entry point is `docker compose up`, and the team ID/token are
# entered in docker-compose.yml.
#
# Build context = the starter kit root (copies marc_sdk + demo together).
#   -> see compose build.context=.. , dockerfile=demo/Dockerfile.
# ---------------------------------------------------------------------------
ARG DEBIAN_FRONTEND=noninteractive

# Ubuntu 22.04 + ROS2 Humble + Python 3.10 (provides rclpy / *_msgs, default RMW=FastDDS)
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Participant algorithm dependencies. numpy + pyyaml (stdlib/ROS2 for the rest), plus placo
# for the closed-loop arm IK pick (Pinocchio-based, pip-installable on Python 3.10).
# The network is used at build time only -- the judging runtime environment has no internet (no runtime dependency).
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-numpy python3-yaml python3-pip \
    && pip3 install --no-cache-dir placo \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /agent

# context = starter kit root -> copy marc_sdk (library) + demo (agent)
COPY marc_sdk /agent/marc_sdk
COPY demo     /agent/demo

# Same RMW as the runtime platform (FastDDS). Import paths for marc_sdk (/agent) + demo local modules (/agent/demo).
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV PYTHONPATH=/agent:/agent/demo

WORKDIR /agent/demo

# The ros:humble ENTRYPOINT (/ros_entrypoint.sh) sources /opt/ros/humble/setup.bash
# and then runs the CMD below. team_id/token are injected via the compose env (from_env).
CMD ["python3", "participant_app.py"]
