"""MARC 2026 ROS2 protocol constants - message IDs, QoS profiles, topic-name builder.

These values mirror the message protocol described in the developer guide's
API Reference (message dictionary, handshake, and QoS profiles).
"""

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)


# -- Message IDs (see the API Reference message dictionary) --
# Participant -> Platform
MSG_SESSION_HELLO = 100             # /marc/ops/register - handshake step 1 (challenge request)
MSG_SESSION_REGISTER = 100          # alias for SESSION_HELLO
MSG_SESSION_PROOF = 101             # /marc/ops/register - handshake step 3 (HMAC proof)
MSG_GROUNDING_RESULT = 301          # /marc/ops/{team}/request
MSG_TASK_COMPLETE = 302             # /marc/ops/{team}/request
MSG_STAGE2_GROUNDING_RESULT = 311   # /marc/ops/{team}/request

# Platform -> All (announce)
MSG_MISSION_COMMAND = 201
MSG_COMPETITION_STATE = 202
MSG_TIME_REMAINING = 203
MSG_TIME_EXPIRED = 204
MSG_STAGE2_MISSION = 211

# Platform -> Participant (response)
MSG_SESSION_CHALLENGE = 410         # handshake step 2 (server_nonce issuance)
MSG_SESSION_ACK = 400               # handshake step 4 (session_key issuance)
MSG_SCORE_RESULT = 401
MSG_STAGE2_REVEAL = 411             # after Stage2 grounding scoring, reveal (score + approximate location + type)

# Platform -> Participant (notification)
MSG_STAGE_TRANSITION = 501
MSG_WARNING = 502


# -- Competition state (msg 202 COMPETITION_STATE) --
STATE_READY = "READY"
STATE_STAGE1_RUN = "STAGE1_RUN"
STATE_STAGE2_RUN = "STAGE2_RUN"
STATE_FINISHING = "FINISHING"
STATE_FINISHED = "FINISHED"


# -- QoS profiles (see the API Reference QoS profiles) --

# Operation messages (JSON), cmd_vel, joint_command, TF/Clock - RELIABLE / VOLATILE
QOS_RELIABLE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# {team}/response, map (occupancy) - RELIABLE / TRANSIENT_LOCAL (late-joiner guarantee)
QOS_TRANSIENT = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# Sensor images (Image) - BEST_EFFORT / VOLATILE / depth=1 (latest frame only)
QOS_IMAGE = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


# -- Robot camera 'which' -> topic suffix mapping --
# Follows the namespace tree in the API Reference (base_camera/ and gripper_camera/).
ROBOT_CAMERA_SUFFIX = {
    "base_left": "base_camera/left",
    "base_right": "base_camera/right",
    "gripper_left": "gripper_camera/left",
    "gripper_right": "gripper_camera/right",
}
ROBOT_DEPTH_SUFFIX = {
    "base": "base_camera/depth",
    "gripper": "gripper_camera/depth",
}


class Topics:
    """Builder that constructs full topic paths from a namespace and team_id.

    Example:
        t = Topics("marc", "alpha")
        t.ops_register            # /marc/ops/register
        t.ops_team_request        # /marc/ops/alpha/request
        t.robot("cmd_vel")        # /marc/alpha/robot/cmd_vel
        t.env_cctv_image("rig_1_a")  # /marc/env/cctv/rig_1_a/image
    """

    def __init__(self, namespace: str = "marc", team_id: str = ""):
        self.ns = namespace.strip("/")
        self.team_id = team_id

    # -- Operations (ops) --
    @property
    def ops_register(self) -> str:
        return f"/{self.ns}/ops/register"

    @property
    def ops_announce(self) -> str:
        return f"/{self.ns}/ops/announce"

    @property
    def ops_team_request(self) -> str:
        return f"/{self.ns}/ops/{self.team_id}/request"

    @property
    def ops_team_response(self) -> str:
        return f"/{self.ns}/ops/{self.team_id}/response"

    @property
    def ops_team_notification(self) -> str:
        return f"/{self.ns}/ops/{self.team_id}/notification"

    # -- Environment (env) --
    @property
    def env_cctv_prefix(self) -> str:
        return f"/{self.ns}/env/cctv/"

    def env_cctv_image(self, camera_id: str) -> str:
        return f"/{self.ns}/env/cctv/{camera_id}/image"

    def env_cctv_info(self, camera_id: str) -> str:
        return f"/{self.ns}/env/cctv/{camera_id}/info"

    @property
    def env_map_occupancy(self) -> str:
        return f"/{self.ns}/env/map/occupancy"

    @property
    def env_map_metadata(self) -> str:
        return f"/{self.ns}/env/map/metadata"

    # -- Robot (team namespace) --
    def robot(self, suffix: str) -> str:
        return f"/{self.ns}/{self.team_id}/robot/{suffix.lstrip('/')}"
