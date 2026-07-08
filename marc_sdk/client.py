"""MARCClient - MARC 2026 participant-facing ROS2 protocol abstraction client.

Hides protocol boilerplate such as handshaking (register->ack), msg_id
dispatch, seq/session pairing, QoS, dynamic CCTV discovery, and world_pose->yaw
conversion. Participants register event handlers via decorators and only use the
submit/control/sensor methods.

See the developer guide's API Reference for the message protocol.

Basic usage:

    from marc_sdk import MARCClient

    client = MARCClient.from_env()          # MARC_TEAM_ID / MARC_TOKEN
    client.connect()                        # register -> ack handled automatically

    @client.on_mission
    def handle(mission):
        result = my_vla.process(mission)    # participant algorithm
        client.submit_grounding(result)

    @client.on_stage2_mission
    def handle2(mission):
        client.submit_stage2_grounding(my_vla.process(mission))

    @client.on_stage2_run
    def drive():
        while client.is_running:
            client.send_cmd_vel(linear_x=0.3)
        client.task_complete()

    client.run()
"""

import json
import logging
import math
import os
import secrets
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, CameraInfo, Imu, LaserScan, JointState
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid

from . import auth
from . import protocol as P
from .protocol import Topics
from .types import Mission, Stage2Mission, Score, Stage2Reveal, GroundingResult

log = logging.getLogger("marc_sdk")


def _build_header(msg_id, seq=None, session=None) -> dict:
    header = {"msg": msg_id, "timestamp": time.time()}
    if seq is not None:
        header["seq"] = seq
    if session is not None:
        header["session"] = session
    return header


class _ParticipantNode(Node):
    """Internal rclpy node. Owned by MARCClient and not exposed directly."""

    def __init__(self, client: "MARCClient", node_name: str):
        from rclpy.parameter import Parameter
        # Follow the platform's simulation clock (/clock) so participant timing
        # (nav timeout / stall detection / arm keyframe holds) is measured in SIM
        # time, not wall time. Under a real-time factor < 1 (heavy render / GUI /
        # loaded scoring HW), wall-clock pacing would false-timeout the nav or
        # advance arm keyframes before the robot physically reaches them. rclpy's
        # TimeSource then subscribes to /clock and drives get_clock().now().
        super().__init__(
            node_name,
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, True),
            ],
        )
        self._client = client


class MARCClient:
    """Participant client that communicates with the MARC 2026 platform.

    Threading model:
        - ``run()`` spins the node with a MultiThreadedExecutor (blocks the main thread).
        - Callback handlers (on_mission, etc.) are invoked on the executor thread.
        - The ``on_stage2_run`` handler is invoked once on a **separate daemon
          thread** when STAGE2_RUN is entered. This way, the
          ``while client.is_running:`` blocking loop inside the handler does not
          block the executor's sensor reception / pose updates.
        - Every sensor getter returns the latest value from a lock-protected
          cache (thread-safe).
    """

    def __init__(
        self,
        team_id: str = None,
        team: str = None,
        token: str = None,
        namespace: str = "marc",
        node_name: str = "marc_participant",
    ):
        self.team_id = (team_id if team_id is not None
                        else os.environ.get("MARC_TEAM_ID", "")).strip()
        # token = your team secret. Used only as the HMAC key in the handshake
        # and never transmitted. The only value on the wire is the session_key
        # issued by the platform.
        self.token = (token if token is not None
                      else os.environ.get("MARC_TOKEN", "")).strip()
        self.team = team if team is not None else self.team_id
        if not self.team_id:
            raise ValueError(
                "team_id is empty. Pass it as an argument or set the MARC_TEAM_ID environment variable."
            )
        if not self.token:
            raise ValueError(
                "token is empty. Pass it as an argument or set the MARC_TOKEN environment variable."
            )

        self.topics = Topics(namespace, self.team_id)
        self._node_name = node_name

        # State
        self._state = P.STATE_READY
        self._registered = False
        self._register_event = threading.Event()
        # challenge-response session authentication state
        self._client_nonce = ""        # regenerated on each connect()
        self._session_key = ""         # expiring key issued via SESSION_ACK
        self._session_expires_at = None
        self._seq = 0
        self._seq_lock = threading.Lock()
        self._stage2_thread = None
        self._stage2_launched = False
        self._stage2_active = False   # whether the Stage 2 driving segment has been entered (is_running guard)
        self._shutdown = False

        # Callback handlers (registered via decorators)
        self._cb_mission = None
        self._cb_stage2_mission = None
        self._cb_stage2_run = None
        self._cb_state_change = None
        self._cb_time_remaining = None
        self._cb_time_expired = None
        self._cb_score = None
        self._cb_stage2_reveal = None
        self._cb_transition = None
        self._cb_warning = None
        self._stage2_reveal = None     # most recently received Stage2Reveal (msg 411) - for polling

        # Sensor cache (thread-safe)
        self._lock = threading.Lock()
        self._cctv_image = {}     # camera_id -> Image
        self._cctv_info = {}      # camera_id -> CameraInfo
        self._cctv_subs = set()   # already-subscribed camera_id
        self._robot_msgs = {}     # suffix -> latest message
        self._robot_subs = set()  # robot suffix already lazily subscribed
        self._world_pose = None   # (x, y, yaw)
        self._occupancy = None    # OccupancyGrid

        # rclpy
        self._node = None
        self._executor = None
        self._pub_cmd_vel = None

    # -- Factory --

    @classmethod
    def from_env(cls, **kwargs) -> "MARCClient":
        """Create from the MARC_TEAM_ID / MARC_TOKEN environment variables."""
        return cls(**kwargs)

    # -- Decorators (handler registration) --

    def on_mission(self, fn):
        """Stage 1 mission command (msg 201). Handler signature: ``fn(mission: Mission)``."""
        self._cb_mission = fn
        return fn

    def on_stage2_mission(self, fn):
        """Stage 2 mission command (msg 211). Signature: ``fn(mission: Stage2Mission)``."""
        self._cb_stage2_mission = fn
        return fn

    def on_stage2_run(self, fn):
        """Invoked once on a separate thread when STAGE2_RUN is entered. Signature: ``fn()``.

        Inside the handler, drive with a ``while client.is_running:`` loop and,
        after it finishes, call ``client.task_complete()``.
        """
        self._cb_stage2_run = fn
        return fn

    def on_state_change(self, fn):
        """Competition state transition (msg 202). Signature: ``fn(old: str, new: str)``."""
        self._cb_state_change = fn
        return fn

    def on_time_remaining(self, fn):
        """Remaining time (msg 203). Signature: ``fn(remaining: float)``."""
        self._cb_time_remaining = fn
        return fn

    def on_time_expired(self, fn):
        """Time expired (msg 204). Signature: ``fn(which: str)`` - 'stage1' | 'total'."""
        self._cb_time_expired = fn
        return fn

    def on_score(self, fn):
        """Scoring result (msg 401). Signature: ``fn(score: Score)``."""
        self._cb_score = fn
        return fn

    def on_stage2_reveal(self, fn):
        """Reveal after Stage 2 grounding scoring (msg 411). Signature: ``fn(reveal: Stage2Reveal)``.

        Right after grounding submission, the platform sends the score + the
        *approximate* location of the correct object (hint_center/hint_radius) +
        the type of object to pick (target_type). If pick/delivery driving
        targets this approximate location, it can be evaluated regardless of
        grounding accuracy. You may also poll via the ``client.stage2_reveal``
        property.
        """
        self._cb_stage2_reveal = fn
        return fn

    def on_transition(self, fn):
        """Stage transition notification (msg 501). Signature: ``fn(from_state, to_state)``."""
        self._cb_transition = fn
        return fn

    def on_warning(self, fn):
        """Warning notification (msg 502). Signature: ``fn(type: str, message: str)``."""
        self._cb_warning = fn
        return fn

    # -- Connection --

    def connect(self, timeout: float = 30.0, register_period: float = 2.0) -> bool:
        """rclpy init -> create node/pub-sub -> handshake -> wait for SESSION_ACK.

        challenge-response handshake (see the API Reference handshake section):
          1) Publish HELLO (msg 100, team_id+client_nonce) - the token is not sent.
          2) The platform replies with SESSION_CHALLENGE (msg 410, server_nonce).
          3) ``_on_response`` computes proof=HMAC(token, ...) and publishes PROOF (msg 101).
          4) The platform replies with SESSION_ACK (msg 400, session_key, expires_at) -> registration complete.

        Re-publishes HELLO every ``register_period`` seconds to self-recover from
        lost challenge/proof messages, and returns True if an ack arrives within
        ``timeout`` seconds, otherwise False.
        """
        if not rclpy.ok():
            rclpy.init()

        # Reset handshake state - client_nonce must be finalized before
        # _setup_subscribers. Because the response topic is TRANSIENT_LOCAL, a
        # retained (stale) message arrives at the spin-thread callback
        # immediately upon subscription; at that moment _client_nonce must
        # already be the current value so that stale messages (with a different
        # nonce) can be filtered out accurately. (Keeping the same value across
        # HELLO re-publishes -> the platform reuses the same challenge, making the
        # handshake idempotent.)
        self._registered = False
        self._register_event.clear()
        self._client_nonce = secrets.token_hex(16)

        self._node = _ParticipantNode(self, self._node_name)
        self._setup_publishers()
        self._setup_subscribers()

        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin, daemon=True,
        )
        self._spin_thread.start()

        # HELLO re-publish loop (until ack is received)
        deadline = time.time() + timeout
        self._log("[REGISTER] starting handshake to %s", self.topics.ops_register)
        while time.time() < deadline:
            self._publish_hello()
            if self._register_event.wait(register_period):
                self._log("[ACK] registration succeeded (team_id=%s, session issued)", self.team_id)
                return True
        self._log_warn("[REGISTER] no SESSION_ACK within %.0fs - check token/runtime", timeout)
        return False

    def _setup_publishers(self):
        t = self.topics
        self._pub_register = self._node.create_publisher(
            String, t.ops_register, P.QOS_RELIABLE)
        self._pub_request = self._node.create_publisher(
            String, t.ops_team_request, P.QOS_RELIABLE)
        self._pub_cmd_vel = self._node.create_publisher(
            Twist, t.robot("cmd_vel"), P.QOS_RELIABLE)
        self._pub_arm = self._node.create_publisher(
            JointState, t.robot("arm/joint_command"), P.QOS_RELIABLE)

    def _setup_subscribers(self):
        t = self.topics
        n = self._node
        n.create_subscription(String, t.ops_announce, self._on_announce, P.QOS_RELIABLE)
        n.create_subscription(String, t.ops_team_response, self._on_response, P.QOS_TRANSIENT)
        n.create_subscription(String, t.ops_team_notification, self._on_notification, P.QOS_RELIABLE)
        # dynamic CCTV discovery
        n.create_timer(2.0, self._discover_cctv)
        # world_pose / occupancy (Stage 2 nav assistance)
        n.create_subscription(PoseStamped, t.robot("world_pose"), self._on_world_pose, P.QOS_RELIABLE)
        n.create_subscription(OccupancyGrid, t.env_map_occupancy, self._on_occupancy, P.QOS_TRANSIENT)
        # grasp-hold / basket-presence feedback (Stage 2 manipulation) -- subscribe
        # eagerly so is_grasping() / is_basket_occupied() have a value on the first call
        # (a lazy first read right after subscribing races the first message -> None).
        self._ensure_robot_sub("gripper/holding", Bool, P.QOS_RELIABLE)
        self._ensure_robot_sub("basket/occupied", Bool, P.QOS_RELIABLE)

    def _publish_hello(self):
        """Handshake step 1 - HELLO (msg 100). Requests a challenge without sending the token."""
        msg = String()
        msg.data = json.dumps({
            "header": _build_header(P.MSG_SESSION_HELLO),
            "payload": {"team_id": self.team_id, "team": self.team,
                        "client_nonce": self._client_nonce},
        }, ensure_ascii=False)
        self._pub_register.publish(msg)

    def _publish_proof(self, server_nonce: str):
        """Handshake step 3 - PROOF (msg 101). Sends only the proof, using token as the HMAC key."""
        proof = auth.hmac_proof(self.token, server_nonce, self._client_nonce,
                                self.team_id)
        msg = String()
        msg.data = json.dumps({
            "header": _build_header(P.MSG_SESSION_PROOF),
            "payload": {"team_id": self.team_id, "server_nonce": server_nonce,
                        "proof": proof},
        }, ensure_ascii=False)
        self._pub_register.publish(msg)

    # -- Receive callbacks (executor thread) --

    def _on_announce(self, msg: String):
        data = self._parse(msg)
        if data is None:
            return
        mid = data["header"].get("msg")
        payload = data["payload"]
        if mid == P.MSG_MISSION_COMMAND:
            self._dispatch(self._cb_mission, Mission.from_payload(payload))
        elif mid == P.MSG_STAGE2_MISSION:
            # Entering Stage 2. Because announce uses VOLATILE QoS, msg 202
            # (STAGE2_RUN) may be missed, so the receipt of 211 itself is treated
            # as a reliable signal for the driving segment.
            self._stage2_active = True
            self._dispatch(self._cb_stage2_mission, Stage2Mission.from_payload(payload))
            # Launch the driving thread only after the interpretation handler
            # (grounding submission) has finished.
            self._launch_stage2_run()
        elif mid == P.MSG_COMPETITION_STATE:
            self._handle_state(payload.get("state", P.STATE_READY))
        elif mid == P.MSG_TIME_REMAINING:
            self._dispatch(self._cb_time_remaining, float(payload.get("remaining", 0.0)))
        elif mid == P.MSG_TIME_EXPIRED:
            self._dispatch(self._cb_time_expired, payload.get("expired"))

    def _on_response(self, msg: String):
        data = self._parse(msg)
        if data is None:
            return
        header, payload = data["header"], data["payload"]
        mid = header.get("msg")
        # The response topic is TRANSIENT_LOCAL (depth=10) - on restart, a stale
        # CHALLENGE/ACK from the previous session is delivered as retained. If the
        # client_nonce echoed by the platform differs from the current connect's,
        # it is stale and discarded (if there is no echo -> old version -> allowed).
        _cn = payload.get("client_nonce")
        if _cn and _cn != self._client_nonce:
            return
        if mid == P.MSG_SESSION_CHALLENGE:
            # Received handshake step 2 -> compute proof and publish step 3.
            server_nonce = payload.get("server_nonce")
            if server_nonce and not self._registered:
                self._publish_proof(server_nonce)
        elif mid == P.MSG_SESSION_ACK:
            if payload.get("status") == "ok":
                self._session_key = payload.get("session_key", "")
                self._session_expires_at = payload.get("expires_at")
                if not self._session_key:
                    self._log_warn("[ACK] session_key missing - check runtime version")
                self._registered = True
                self._register_event.set()
            else:
                self._log_warn("[ACK] registration failed: %s", payload.get("status_message"))
        elif mid == P.MSG_SCORE_RESULT:
            self._dispatch(self._cb_score, Score.from_payload(payload, header.get("seq")))
        elif mid == P.MSG_STAGE2_REVEAL:
            reveal = Stage2Reveal.from_payload(payload)
            self._stage2_reveal = reveal
            self._dispatch(self._cb_stage2_reveal, reveal)

    def _on_notification(self, msg: String):
        data = self._parse(msg)
        if data is None:
            return
        mid, payload = data["header"].get("msg"), data["payload"]
        if mid == P.MSG_STAGE_TRANSITION:
            self._dispatch(self._cb_transition, payload.get("from"), payload.get("to"))
        elif mid == P.MSG_WARNING:
            self._dispatch(self._cb_warning, payload.get("type"), payload.get("message"))

    def _handle_state(self, new_state: str):
        old = self._state
        if new_state == old:
            return
        self._state = new_state
        self._log("[STATE] %s -> %s", old, new_state)
        self._dispatch(self._cb_state_change, old, new_state)
        if new_state == P.STATE_STAGE2_RUN:
            self._stage2_active = True
            # If there is no on_stage2_mission handler (= a participant who skips
            # Stage 2 interpretation), launch driving on the state transition
            # alone. If a handler exists, launch it via the 211 path so that
            # grounding submission happens before driving.
            if self._cb_stage2_mission is None:
                self._launch_stage2_run()

    def _launch_stage2_run(self):
        """Launch the on_stage2_run handler once on a daemon thread (safe against duplicate calls)."""
        if self._cb_stage2_run is None or self._stage2_launched:
            return
        self._stage2_launched = True
        self._stage2_thread = threading.Thread(
            target=self._run_stage2_handler, daemon=True)
        self._stage2_thread.start()

    def _run_stage2_handler(self):
        try:
            self._cb_stage2_run()
        except Exception as e:  # noqa: BLE001
            self._log_err("on_stage2_run handler exception: %s", e)

    # -- Sensor callbacks --

    def _discover_cctv(self):
        prefix = self.topics.env_cctv_prefix
        for name, _types in self._node.get_topic_names_and_types():
            if not name.startswith(prefix):
                continue
            remainder = name[len(prefix):]
            if remainder.endswith("/image"):
                cid = remainder[:-len("/image")]
                if cid and cid not in self._cctv_subs:
                    self._cctv_subs.add(cid)
                    self._node.create_subscription(
                        Image, name,
                        lambda m, c=cid: self._cache_cctv_image(c, m), P.QOS_IMAGE)
                    self._node.create_subscription(
                        CameraInfo, self.topics.env_cctv_info(cid),
                        lambda m, c=cid: self._cache_cctv_info(c, m), P.QOS_IMAGE)
                    self._log("[SUB] CCTV %s", cid)

    def _cache_cctv_image(self, cid, msg):
        with self._lock:
            self._cctv_image[cid] = msg

    def _cache_cctv_info(self, cid, msg):
        with self._lock:
            self._cctv_info[cid] = msg

    def _on_world_pose(self, msg: PoseStamped):
        p, q = msg.pose.position, msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        with self._lock:
            self._world_pose = (float(p.x), float(p.y), float(yaw))

    def _on_occupancy(self, msg: OccupancyGrid):
        with self._lock:
            self._occupancy = msg

    def _cache_robot(self, suffix, msg):
        with self._lock:
            self._robot_msgs[suffix] = msg

    def _ensure_robot_sub(self, suffix, msg_type, qos):
        """Lazily subscribe to a robot topic (once, on the first getter call)."""
        with self._lock:
            if suffix in self._robot_subs:
                return
            self._robot_subs.add(suffix)
        self._node.create_subscription(
            msg_type, self.topics.robot(suffix),
            lambda m, s=suffix: self._cache_robot(s, m), qos)
        self._log("[SUB] robot/%s", suffix)

    # -- Submit (Participant -> Platform) --

    def _next_seq(self) -> int:
        with self._seq_lock:
            self._seq += 1
            return self._seq

    def _publish_request(self, msg_id, payload) -> int:
        seq = self._next_seq()
        msg = String()
        # session field = issued session_key (not a static token). The platform
        # verifies the key's existence/expiry/team-binding and seq monotonicity.
        msg.data = json.dumps({
            "header": _build_header(msg_id, seq=seq, session=self._session_key),
            "payload": payload,
        }, ensure_ascii=False)
        self._pub_request.publish(msg)
        return seq

    def _grounding_payload(self, result) -> dict:
        if not isinstance(result, GroundingResult):
            raise TypeError(
                "result must be a GroundingResult. Build it from its fields, "
                "or from a payload dict via GroundingResult.from_payload(...).")
        return result.to_payload()

    def submit_grounding(self, result: GroundingResult) -> int:
        """Submit the Stage 1 VLA interpretation result (msg 301). Returns: the assigned seq.

        ``result`` must be a ``GroundingResult`` (single-target model: ``target_type``
        is a single string and coordinates are 3D ``[x, y, z]``). For lost items set
        ``relation``; for persons set ``situation``. A payload dict can be converted
        with ``GroundingResult.from_payload(...)``.
        """
        payload = self._grounding_payload(result)
        seq = self._publish_request(P.MSG_GROUNDING_RESULT, payload)
        self._log("[SUBMIT] grounding (msg 301, seq=%d)", seq)
        return seq

    def submit_stage2_grounding(self, result: GroundingResult) -> int:
        """Submit the Stage 2 VLA interpretation result (msg 311). Returns: the assigned seq."""
        payload = self._grounding_payload(result)
        seq = self._publish_request(P.MSG_STAGE2_GROUNDING_RESULT, payload)
        self._log("[SUBMIT] stage2 grounding (msg 311, seq=%d)", seq)
        return seq

    def task_complete(self) -> int:
        """Publish Stage 2 collection complete (msg 302). Transitions to FINISHING immediately, cannot be canceled."""
        seq = self._publish_request(P.MSG_TASK_COMPLETE, {})
        self._log("[SUBMIT] task_complete (msg 302, seq=%d)", seq)
        return seq

    # -- Robot control --

    def send_cmd_vel(self, linear_x: float = 0.0, linear_y: float = 0.0,
                     angular_z: float = 0.0, twist: Twist = None):
        """Publish cmd_vel. If ``twist`` is given, publish it as-is; otherwise build it from the arguments.

        Max linear velocity 1.5 m/s, max angular velocity 1.5 rad/s (see the API Reference control topics).
        """
        if twist is None:
            twist = Twist()
            twist.linear.x = float(linear_x)
            twist.linear.y = float(linear_y)
            twist.angular.z = float(angular_z)
        self._pub_cmd_vel.publish(twist)

    def stop(self):
        """Stop the robot (cmd_vel 0)."""
        self._pub_cmd_vel.publish(Twist())

    def send_arm_command(self, joint_state: JointState):
        """Publish a robot-arm joint command (arm/joint_command) (passthrough)."""
        self._pub_arm.publish(joint_state)

    # -- Sensor access (raw ROS2 messages, latest values) --

    def list_cctv(self):
        """List of discovered CCTV camera IDs."""
        with self._lock:
            return sorted(self._cctv_subs)

    def get_cctv_image(self, camera_id: str):
        """Latest CCTV RGB image frame (sensor_msgs/Image) or None."""
        with self._lock:
            return self._cctv_image.get(camera_id)

    def get_cctv_info(self, camera_id: str):
        """CCTV camera parameters (sensor_msgs/CameraInfo) or None."""
        with self._lock:
            return self._cctv_info.get(camera_id)

    def get_robot_image(self, which: str = "base_left"):
        """Robot stereo RGB (sensor_msgs/Image). which in base_left/right, gripper_left/right."""
        suffix = P.ROBOT_CAMERA_SUFFIX.get(which)
        if suffix is None:
            raise ValueError(f"Unknown camera '{which}'. Choose one of {list(P.ROBOT_CAMERA_SUFFIX)}.")
        self._ensure_robot_sub(f"{suffix}/image", Image, P.QOS_IMAGE)
        return self._get_robot(f"{suffix}/image")

    def get_robot_depth(self, which: str = "base"):
        """Robot depth image (32FC1, sensor_msgs/Image). which in base/gripper."""
        suffix = P.ROBOT_DEPTH_SUFFIX.get(which)
        if suffix is None:
            raise ValueError(f"Unknown depth '{which}'. Choose one of {list(P.ROBOT_DEPTH_SUFFIX)}.")
        self._ensure_robot_sub(f"{suffix}/image", Image, P.QOS_IMAGE)
        return self._get_robot(f"{suffix}/image")

    def get_odom(self):
        """Odometry (nav_msgs/Odometry) or None."""
        self._ensure_robot_sub("odom", Odometry, P.QOS_RELIABLE)
        return self._get_robot("odom")

    def get_imu(self):
        """IMU (sensor_msgs/Imu) or None."""
        self._ensure_robot_sub("imu", Imu, P.QOS_RELIABLE)
        return self._get_robot("imu")

    def get_lidar(self):
        """2D lidar scan (sensor_msgs/LaserScan) or None."""
        self._ensure_robot_sub("lidar/scan", LaserScan, P.QOS_IMAGE)
        return self._get_robot("lidar/scan")

    def get_arm_state(self):
        """Robot-arm joint state (sensor_msgs/JointState) or None."""
        self._ensure_robot_sub("arm/joint_states", JointState, P.QOS_RELIABLE)
        return self._get_robot("arm/joint_states")

    def is_grasping(self):
        """Whether the gripper is currently holding an object: ``True``/``False`` or None.

        Derived from gripper closure stall, like the Robotiq 2F-140 object-detection
        status: ``True`` while a close command is blocked short of its target by an
        object between the fingers. Use it to detect a failed pick (stays False) or a
        drop during placement (True -> False) and trigger a retry. Poll each step.
        Returns None until the first message arrives.
        """
        self._ensure_robot_sub("gripper/holding", Bool, P.QOS_RELIABLE)
        msg = self._get_robot("gripper/holding")
        return None if msg is None else bool(msg.data)

    def is_basket_occupied(self):
        """Whether an object is currently in the rear basket: ``True``/``False`` or None.

        Generic presence (any graspable object in the basket volume), like a real
        basket presence sensor - it does NOT reveal whether the correct target was
        delivered or your score. Use it to confirm a delivery or detect that the
        object fell outside the basket (retry). Published during Stage 2 only; None
        until the first message arrives.
        """
        self._ensure_robot_sub("basket/occupied", Bool, P.QOS_RELIABLE)
        msg = self._get_robot("basket/occupied")
        return None if msg is None else bool(msg.data)

    def get_occupancy_map(self):
        """Static occupancy grid map (nav_msgs/OccupancyGrid) or None."""
        with self._lock:
            return self._occupancy

    def get_world_pose(self):
        """Robot world-frame pose ``(x, y, yaw_rad)`` or None.

        Same frame as the grounding target_coord. Pose feedback for Stage 2 navigation.
        """
        with self._lock:
            return self._world_pose

    def subscribe(self, topic: str, msg_type, callback, qos=None):
        """Escape hatch to subscribe directly to an arbitrary topic outside the spec. Creates a raw rclpy subscription."""
        return self._node.create_subscription(
            msg_type, topic, callback, qos or P.QOS_RELIABLE)

    def _get_robot(self, suffix):
        with self._lock:
            return self._robot_msgs.get(suffix)

    # -- Lifecycle --

    # Terminal states at which driving must end. Accounting for the UNKNOWN/READY
    # initial values and a missed STAGE2_RUN, we treat "after entering Stage 2
    # (_stage2_active), if not in a terminal state, then driving is in progress".
    _TERMINAL_STATES = (P.STATE_FINISHING, P.STATE_FINISHED)

    @property
    def is_running(self) -> bool:
        """True during the Stage 2 driving segment. Used as the loop guard in on_stage2_run.

        Becomes True on receiving msg 211 (STAGE2_MISSION) or on entering
        STAGE2_RUN, and becomes False on the FINISHING/FINISHED transition or on
        shutdown. (Because announce uses VOLATILE QoS, driving starts on the 211
        signal even if msg 202 is missed.)
        """
        return (self._stage2_active and not self._shutdown
                and self._state not in self._TERMINAL_STATES)

    @property
    def stage2_reveal(self):
        """Most recently received Stage 2 reveal (msg 411) or None. For polling instead of on_stage2_reveal.

        Filled only after grounding submission has been scored - if the
        on_stage2_run driving thread waits briefly and polls, it can use the
        approximate location as the pick target.
        """
        return self._stage2_reveal

    @property
    def state(self) -> str:
        return self._state

    @property
    def registered(self) -> bool:
        return self._registered

    @property
    def node(self):
        """Internal rclpy node (advanced use)."""
        return self._node

    # -- simulation-time clock (robust to real-time factor < 1) --

    def now_s(self) -> float:
        """Current SIM time in seconds, following the platform ``/clock``.

        Use this instead of ``time.time()`` for any motion timing (nav timeout,
        stall detection, arm keyframe holds). If ``/clock`` has not been received
        yet (clock reads 0), it falls back to wall time so early bootstrap logic
        still progresses.
        """
        if self._node is None:
            return time.time()
        ns = self._node.get_clock().now().nanoseconds
        if ns == 0:
            return time.time()  # /clock not received yet -> wall-time fallback
        return ns * 1e-9

    def sleep(self, seconds: float):
        """Sleep for ``seconds`` of SIM time (not wall time).

        Polls ``/clock`` so the wait scales with the real-time factor: at RTF 0.6
        a 1.0s sim sleep takes ~1.67s wall. Returns early if driving stops
        (``is_running`` False). Falls back to a plain wall sleep when ``/clock``
        is unavailable.
        """
        if seconds <= 0:
            return
        start_sim = self.now_s()
        wall_start = time.time()
        # Never hang: if the sim clock stalls (paused / frozen) or /clock is absent, give up
        # after this much WALL time and return (degrade to ~wall pacing). Generous vs RTF so
        # normal sim-time pacing (which finishes in seconds/RTF wall) always wins first.
        wall_cap = max(seconds * 5.0, seconds + 1.0)
        while not self._shutdown:
            if self.now_s() - start_sim >= seconds:
                return                        # sim advanced enough -> sim-time paced
            if time.time() - wall_start >= wall_cap:
                return                        # sim clock stalled/absent -> don't hang
            if self._stage2_active and not self.is_running:
                return
            time.sleep(0.005)

    def run(self):
        """Block the main thread while spinning the node. Exit with Ctrl-C."""
        if self._executor is None:
            raise RuntimeError("Call connect() first.")
        try:
            # The spin thread from connect() is already running the executor,
            # so here we just wait for the shutdown signal.
            while rclpy.ok() and not self._shutdown:
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    # Alias
    spin = run

    def shutdown(self):
        """Send a stop command, then clean up the node/executor (safe against duplicate calls)."""
        if self._shutdown:
            return
        self._shutdown = True
        try:
            if self._pub_cmd_vel is not None:
                self.stop()
        except Exception:  # noqa: BLE001
            pass
        try:
            if self._executor is not None:
                self._executor.shutdown()
            if self._node is not None:
                self._node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
        self._log("[SHUTDOWN] done")

    # -- Internal utilities --

    def _dispatch(self, fn, *args):
        if fn is None:
            return
        try:
            fn(*args)
        except Exception as e:  # noqa: BLE001
            self._log_err("handler exception: %s", e)

    def _parse(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self._log_err("JSON parse failed: %s", e)
            return None
        if "header" not in data or "payload" not in data:
            self._log_warn("header/payload missing: %s", msg.data[:80])
            return None
        return data

    def _log(self, fmt, *args):
        if self._node is not None:
            self._node.get_logger().info(fmt % args if args else fmt)
        else:
            log.info(fmt, *args)

    def _log_warn(self, fmt, *args):
        if self._node is not None:
            self._node.get_logger().warning(fmt % args if args else fmt)
        else:
            log.warning(fmt, *args)

    def _log_err(self, fmt, *args):
        if self._node is not None:
            self._node.get_logger().error(fmt % args if args else fmt)
        else:
            log.error(fmt, *args)
