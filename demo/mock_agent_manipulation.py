"""Robot arm pick sequence -- fixed-keyframe pickup motion (joint-space keyframes).

A dependency-free fallback reference: a fixed-keyframe pick that needs no ``placo``. It is
not the current execution path (the demo uses ``arm_pick.py`` from the manipulation kit); it
is kept as an alternative reference. Instead of converting EE waypoints to joint angles with
IK, it **directly interpolates and publishes joint-angle keyframes**, so it runs with only
the standard ROS 2 message types.

Topic / joint convention (see the API Reference):
    arm/joint_command = JointState(name = joint_1..6 + left/right outer_knuckle,
                                   position = radian)
    gripper: 0.0 (open) ~ 0.703 (close)

Sequence: home -> reach toward the object in front and below -> grasp (close gripper) ->
          lift -> carry to the basket (rear) -> release (open gripper) -> home.
"""

from sensor_msgs.msg import JointState

ARM_JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
GRIPPER_JOINT_NAMES = ["left_outer_knuckle_joint", "right_outer_knuckle_joint"]

GRIPPER_OPEN, GRIPPER_CLOSE = 0.0, 0.703  # aligned to the outer_knuckle joint limit 40.3deg (0.703rad)

# Reference pose (rad) -- the arm's default joint target. Folded-arm home.
HOME = [0.0, 0.262, -2.094, 0.0, -0.785, 1.571]
# Reach keyframes computed offline with placo IK (arm_kin) so the gripper
# actually descends to a ground-level object. EE targets in the arm_base frame:
#   PRE   = (0.40, 0.0, -0.155)  approach just above the object
#   GRASP = (0.40, 0.0, -0.29)   at the object on the ground (grasp point)
#   LIFT  = (0.40, 0.0,  0.246)  carry height (object lifted clear of the ground)
# joint_1 stays ~0 (no lateral swing) so the arm stays in the sagittal plane (tip-over safe).
PRE   = [0.0034, 0.9658, -1.9755, 0.0, -0.1997, 1.5744]
GRASP = [0.0034, 1.3131, -1.8183, 0.0, -0.0096, 1.5743]
LIFT  = [0.0034, 0.2313, -1.5737, 0.0, -1.3359, 1.5744]
# Place in the basket -- gently set down above the front of the chassis (keep joint_1=0, no lateral swing).
PLACE = [0.0, 0.35, -1.85, 0.0, -0.70, 1.571]

# Keyframe tuple format: (label, arm_q(rad x6), gripper(rad), hold_s).
# To prevent tip-over, there is no lateral (joint_1) swing -- motion stays in the
# forward-vertical plane. The full sequence home->reach->grasp->lift->place->release->home
# is defined as phases below (_HOME_F / _PICK_F / _DELIVER_F) so the demo can check
# feedback and retry between the pick and the delivery.

_RATE_HZ = 20.0       # command publish rate
_MOVE_T = 1.8         # interpolation time between keyframes (seconds) -- slow (prevents dynamic tip-over)


def _make_js(arm_q, grip):
    js = JointState()
    js.name = list(ARM_JOINT_NAMES) + list(GRIPPER_JOINT_NAMES)
    js.position = ([float(q) for q in arm_q]
                   + [float(grip)] * len(GRIPPER_JOINT_NAMES))
    return js


_MAX_ATTEMPTS = 2     # retry the pick-and-place at most this many times on failure

# Sequence phases -- checked/retried between the pick and the delivery.
_HOME_F = [("home", HOME, GRIPPER_OPEN, 0.4)]
_PICK_F = [
    ("reach",   PRE,   GRIPPER_OPEN,  0.4),   # approach just above the object
    ("descend", GRASP, GRIPPER_OPEN,  0.4),   # lower straight down onto the object
    ("grasp",   GRASP, GRIPPER_CLOSE, 1.0),   # close gripper = pick
    ("lift",    LIFT,  GRIPPER_CLOSE, 0.6),   # lift up (centered)
]
_DELIVER_F = [
    ("place",   PLACE, GRIPPER_CLOSE, 0.5),  # gently to the front basket
    ("release", PLACE, GRIPPER_OPEN,  1.0),  # drop off = open gripper
]


def _move_through(client, cur_q, cur_g, frames, log):
    """Interpolate to each frame over _MOVE_T, publishing arm/joint_command at _RATE_HZ.

    Returns the final (arm_q, gripper) so the next phase can continue smoothly.
    """
    dt = 1.0 / _RATE_HZ
    for (label, arm_q, grip, hold_s) in frames:
        steps = max(1, int(_MOVE_T * _RATE_HZ))
        for s in range(1, steps + 1):
            a = s / steps
            q = [cur_q[i] * (1.0 - a) + arm_q[i] * a for i in range(6)]
            g = cur_g * (1.0 - a) + grip * a
            client.send_arm_command(_make_js(q, g))
            client.sleep(dt)          # SIM-time pacing (aligns keyframe rate with physics_dt)
        cur_q, cur_g = list(arm_q), grip
        log.info("[arm] %s (gripper=%s)", label,
                 "close" if grip > 0.3 else "open")
        if hold_s > 0:
            client.sleep(hold_s)      # hold in SIM time so the arm actually reaches the target
    return cur_q, cur_g


def run_pick_sequence(client, log):
    """Pick-and-place with grasp / delivery feedback and retry (reference pattern).

    The motion is fixed joint keyframes (this mock does not perceive the object), but it
    shows how a participant uses the platform feedback signals to detect failure and retry:
      - after grasping, ``client.is_grasping()`` confirms the object is actually held;
      - after releasing over the basket, ``client.is_basket_occupied()`` confirms delivery.
    Both are realistic sensor-style signals (they do not reveal the correct target or the
    score). ``None`` means the signal has not arrived yet, so the demo does not treat it as
    a failure. See the developer guide (manipulation kit) for details.
    """
    cur_q, cur_g = list(HOME), GRIPPER_OPEN
    cur_q, cur_g = _move_through(client, cur_q, cur_g, _HOME_F, log)

    for attempt in range(1, _MAX_ATTEMPTS + 1):
        # --- pick ---
        cur_q, cur_g = _move_through(client, cur_q, cur_g, _PICK_F, log)
        if client.is_grasping() is False:
            log.info("[arm] pick failed (is_grasping=False) -- retry %d/%d",
                     attempt, _MAX_ATTEMPTS)
            cur_q, cur_g = _move_through(client, cur_q, cur_g, _HOME_F, log)
            continue
        log.info("[arm] grasp confirmed (is_grasping=%s)", client.is_grasping())

        # --- deliver to the basket ---
        cur_q, cur_g = _move_through(client, cur_q, cur_g, _DELIVER_F, log)
        if client.is_basket_occupied() is False:
            log.info("[arm] delivery missed (is_basket_occupied=False) -- retry %d/%d",
                     attempt, _MAX_ATTEMPTS)
            cur_q, cur_g = _move_through(client, cur_q, cur_g, _HOME_F, log)
            continue
        log.info("[arm] delivery confirmed (is_basket_occupied=%s)",
                 client.is_basket_occupied())
        break

    # return to home
    _move_through(client, cur_q, cur_g, _HOME_F, log)
