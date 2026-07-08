#!/bin/bash
# MARC 2026 SDK Demo Participant Launcher
#
# Runs the baseline demo (Stage 1 grounding + Stage 2 cmd_vel A* nav) with marc_sdk.
# The simulation platform must be running first.
#
# Usage:
#   MARC_TOKEN=<token> MARC_TEAM_ID=u1 ./launch.sh      # set env vars directly
#   ./launch.sh u1                                       # if tokens.env exists, u1 -> U1_TOKEN
#   ROS_DOMAIN_ID=0 ./launch.sh u2
#
# ROS2 distribution:
#   The runtime uses IsaacSim's built-in humble FastDDS. The client must also be humble
#   (if a different distribution joins the same DDS domain, the runtime crashes on
#   discovery deserialization).
#   - If /opt/ros/humble exists, use it (system python3).
#   - Otherwise use IsaacSim's built-in humble bridge (IsaacSim python.sh).

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"   # starter kit root (parent of marc_sdk)

# -- team id (required; issued by the organizers, no default) --
USER_ID="${1:-${MARC_TEAM_ID:-}}"
if [ -z "${USER_ID}" ]; then
    echo "ERROR: team id not set. Provide it: './launch.sh <team_id>' or 'MARC_TEAM_ID=<team_id> ./launch.sh'." >&2
    exit 1
fi

# -- team token (required; issued by the organizers, never defaulted or derived locally) --
#   Provide it directly (MARC_TOKEN), or via a tokens.env file (<ID>_TOKEN=...).
if [ -z "${MARC_TOKEN:-}" ] && [ -f "${SCRIPT_DIR}/tokens.env" ]; then
    # shellcheck disable=SC1090
    source "${SCRIPT_DIR}/tokens.env"
    TOKEN_VAR="$(echo "${USER_ID}" | tr '[:lower:]' '[:upper:]')_TOKEN"
    MARC_TOKEN="${!TOKEN_VAR:-${MARC_TOKEN:-}}"
fi
if [ -z "${MARC_TOKEN:-}" ]; then
    echo "ERROR: MARC_TOKEN not set. Provide it: 'MARC_TOKEN=<token> ./launch.sh' or add a tokens.env file." >&2
    exit 1
fi
export MARC_TEAM_ID="${USER_ID}"
export MARC_TOKEN

# -- ROS2 environment: humble (system) or IsaacSim's built-in humble bridge --
RUN_PY="python3"
if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
else
    # If system humble is absent, fall back to IsaacSim's built-in humble bridge (same DDS as the platform).
    ISAAC_SIM_PATH="${ISAAC_SIM_PATH:-${ISAACSIM_PATH:-${HOME}/isaacsim/_build/linux-x86_64/release}}"
    BRIDGE="${ISAAC_SIM_PATH}/exts/isaacsim.ros2.bridge/humble"
    if [ ! -d "${BRIDGE}/lib" ] || [ ! -d "${BRIDGE}/rclpy" ]; then
        echo "ERROR: could not find ROS2 humble."
        echo "  - /opt/ros/humble is absent, and"
        echo "  - the IsaacSim humble bridge is also absent: ${BRIDGE}"
        echo "  Set ISAACSIM_PATH or install ROS2 humble."
        exit 1
    fi
    echo "[launch] system humble absent -> using IsaacSim humble bridge: ${BRIDGE}"
    # Remove other distribution paths (/opt/ros/*, e.g. jazzy) -- mixing them crashes the runtime at discovery.
    export LD_LIBRARY_PATH="$(echo "${LD_LIBRARY_PATH}" | tr ':' '\n' | grep -v '/opt/ros' | tr '\n' ':' | sed 's/:$//')"
    export PYTHONPATH="$(echo "${PYTHONPATH}" | tr ':' '\n' | grep -v '/opt/ros' | tr '\n' ':' | sed 's/:$//')"
    export ROS_DISTRO=humble
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export LD_LIBRARY_PATH="${BRIDGE}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
    export PYTHONPATH="${BRIDGE}/rclpy${PYTHONPATH:+:${PYTHONPATH}}"
    RUN_PY="${ISAAC_SIM_PATH}/python.sh"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# -- PYTHONPATH: marc_sdk (=SDK_ROOT) + demo local modules (=SCRIPT_DIR) --
# (If the SDK is pip installed, SDK_ROOT is unnecessary, but it is kept so the repo runs directly)
export PYTHONPATH="${SDK_ROOT}:${SCRIPT_DIR}${PYTHONPATH:+:${PYTHONPATH}}"

echo "=========================================="
echo "  MARC 2026 SDK Demo Participant"
echo "  team_id:        ${MARC_TEAM_ID}"
echo "  ROS_DISTRO:     ${ROS_DISTRO:-humble}"
echo "  ROS_DOMAIN_ID:  ${ROS_DOMAIN_ID}"
echo "  python:         ${RUN_PY}"
echo "=========================================="

# -- placo (Stage2 pick kinematics via arm_pick) --
#   The demo imports arm_pick, which needs placo. In Docker it is baked into the image; when running
#   directly here, ensure it is present for ${RUN_PY} (install into that same interpreter if missing).
if ! "${RUN_PY}" -c "import placo" >/dev/null 2>&1; then
    echo "[launch] placo not found for ${RUN_PY} -- installing (pip install placo)..."
    if ! "${RUN_PY}" -m pip install placo; then
        echo "ERROR: failed to install placo automatically." >&2
        echo "  Install it manually for this interpreter: ${RUN_PY} -m pip install placo" >&2
        echo "  (on an externally-managed Python you may need --user or --break-system-packages)" >&2
        exit 1
    fi
fi

cd "${SCRIPT_DIR}"
exec "${RUN_PY}" participant_app.py
