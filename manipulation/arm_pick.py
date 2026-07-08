"""Robot-arm pick-and-place - grabs a tumbler and drops it into the rear basket.

It computes the URDF FK/Jacobian with placo (Pinocchio); the EE tip is the URDF `ee_frame_offset`
frame (the midpoint of the two finger pads), and every tick it reads `arm/joint_states` and converges
on the target waypoint. The engine (PlacoArmKinematics/DLS/WaypointSequence) lives in arm_kin.py.

The path is defined by the waypoint table in build_waypoints (format: label, ee, grip, hold, mode):
  pre -> descend -> grasp (close) -> lift -> over (joint arc) -> place -> release (open) -> up
  -> back (joint arc). mode "cart" = straight-line DLS EE tracking (vertically precise), "joint" =
  joint-space interpolation (base-rotation arc, singularity avoidance). The EE orientation stays at
  default_quat throughout. The xy of the grasp point is parameterized by grasp_xy (the object's actual
  arm_base coordinates), and the rest is tuned via parameters (descent depth, transport height, basket,
  speed). Parameters are read from pnp_params.json (FINAL_PARAMS if absent).

Topic/joint convention:
    arm/joint_command = JointState(name = joint_1..6 + left/right outer_knuckle, position=rad)
    gripper: 0.0(open) ~ 0.703(close)
"""

import json
import os
import time

import numpy as np
from sensor_msgs.msg import JointState

import arm_kin

ARM_JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
GRIPPER_JOINT_NAMES = ["left_outer_knuckle_joint", "right_outer_knuckle_joint"]

GRIPPER_OPEN, GRIPPER_CLOSE = 0.0, 0.703  # aligned to the outer_knuckle joint limit 40.3deg (0.703rad)
HOME = [0.0, 0.262, -2.094, 0.0, -0.785, 1.571]  # default pose = arm_kin.DEFAULT_ARM_Q

# URDF (loaded via placo). gen3 6DoF + robotiq 140. (the kit uses the urdf/ subfolder)
_URDF = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "urdf", "gen3_6dof_vision_2f140.urdf")
_EE_FRAME = "ee_frame_offset"

# Default pick parameters (tuned values). Override any of them via pnp_params.json.
# grasp_* = grasp point (xy overridden by grasp_xy), carry_up = transport/rotation height, place_* = basket.
FINAL_PARAMS = {
    "grasp_fwd": 0.40, "grasp_lat": 0.0, "grasp_down": -0.314, "pre_up": 0.135,
    "carry_up": 0.246, "place_fwd": -0.29, "place_lat": 0.0, "place_up": 0.046,
    "grip_close": 0.703, "move_speed": 0.35, "carry_speed": 0.22, "grip_dwell": 1.0,
    "grasp_yaw": 0.52,
}
_PARAMS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pnp_params.json")

# placo kinematics has a URDF-load cost, so cache it once per process.
_KIN = None


def _kin():
    global _KIN
    if _KIN is None:
        _KIN = arm_kin.PlacoArmKinematics(_URDF, _EE_FRAME, list(ARM_JOINT_NAMES))
    return _KIN


def load_params():
    """Read pnp_params.json and return a dict overlaid on top of FINAL_PARAMS."""
    p = dict(FINAL_PARAMS)
    try:
        with open(_PARAMS_FILE) as f:
            saved = json.load(f)
        for k in p:
            if k in saved:
                p[k] = float(saved[k])
    except Exception:
        pass
    return p


def _make_js(arm_q, grip):
    js = JointState()
    js.name = list(ARM_JOINT_NAMES) + list(GRIPPER_JOINT_NAMES)
    js.position = ([float(q) for q in arm_q]
                   + [float(grip)] * len(GRIPPER_JOINT_NAMES))
    return js


def build_waypoints(p, phase="full"):
    """pick-and-place waypoint table - format (label, ee[arm_base], grip, hold, mode).

    Tune by reading and editing this table directly. mode "cart" = straight-line EE tracking (vertically
    precise), "joint" = joint-space interpolation (base-perimeter rotation arc, singularity avoidance).
    The EE orientation stays at default_quat (= fk(DEFAULT_ARM_Q) rotation) throughout. grip is latched
    on reaching, then it waits hold seconds.

    Path: pre (straight up) -> descend (straight down) -> grasp (close) -> lift (straight up)
      -> over (rotate to above the basket, joint arc) -> place (descend to basket) -> release (open)
      -> up (straight up) -> back (rotate back to the front, joint arc).
    phase="pick": up to lift. phase="full": up to place.
    """
    g = [p["grasp_fwd"], p["grasp_lat"], p["grasp_down"]]
    pre = [g[0], g[1], g[2] + p["pre_up"]]
    cz = p["carry_up"]                                    # transport/rotation height (above the base)
    lift = [g[0], g[1], cz]                               # straight up from the grasp position
    over = [p["place_fwd"], p["place_lat"], cz]           # to above the basket at high z
    place = [p["place_fwd"], p["place_lat"], p["place_up"]]  # descend to the basket
    front_high = [0.30, 0.0, cz]                          # return waypoint (high, at the front)
    cl, op, d = p["grip_close"], GRIPPER_OPEN, p["grip_dwell"]
    #      label      ee           grip  hold  mode
    pick = [
        ("pre",     pre,          op,   0.3,  "cart"),   # straight above the target
        ("descend", g,            op,   0.2,  "cart"),   # descend straight down
        ("grasp",   g,            cl,   d,    "cart"),   # close the gripper to grab
        ("lift",    lift,         cl,   0.3,  "cart"),   # straight up (above the base)
    ]
    place_wps = [
        ("over",    over,         cl,   0.3,  "joint"),  # rotate to above the basket (arc)
        ("place",   place,        cl,   0.3,  "cart"),   # descend straight down to the basket
        ("release", place,        op,   d,    "cart"),   # open the gripper to drop
        ("up",      over,         op,   0.2,  "cart"),   # straight up again
        ("back",    front_high,   op,   0.2,  "joint"),  # rotate back to the front (arc)
    ]
    return pick if phase == "pick" else pick + place_wps


class _Log:
    """info/warn adapter expected by arm_kin (WaypointSequence).

    The caller's log may be either a callable (harness) or a logging.Logger.
    """

    def __init__(self, log):
        self._log = log

    def _emit(self, msg):
        if callable(self._log):
            self._log(msg)
        else:
            fn = getattr(self._log, "info", None)
            if fn:
                fn(msg)

    def info(self, msg):
        self._emit(msg)

    def warn(self, msg):
        self._emit("WARN: " + msg)


_DBG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".pnp_debug.log")


def _dbg_open(phase, waypoints, pos_step, joint_step, rate):
    """If MARC_PNP_DEBUG=1, open the servo-diagnostics log file and write the header (else None).

    Per tick it records loop_dt (actual loop period), feedback freshness (fresh/stale), ee_err to the
    current waypoint, and the command increment (cmd_dq) -> numerically distinguishes the cause of
    "sluggishness" (stale feedback / slow loop / convergence oscillation / plain slowness).
    """
    if os.environ.get("MARC_PNP_DEBUG", "0") in ("0", "", "false", "False"):
        return None
    try:
        fh = open(_DBG_PATH, "a")
    except Exception:
        return None
    fh.write(f"\n===== run_motion {phase}  pos_step={pos_step:.4f} "
             f"joint_step={joint_step:.4f} rate={rate:g} =====\n")
    for w in waypoints:
        fh.write(f"  wp {w[0]:<9} ee={[round(float(x),3) for x in w[1]]} "
                 f"grip={w[2]:.2f} hold={w[3]} {w[4]}\n")
    fh.flush()
    return fh


def _read_arm_q(client):
    """Extract joint_1..6 joint angles (rad) from client.get_arm_state(). None if absent."""
    try:
        st = client.get_arm_state()
    except Exception:
        st = None
    if st is None or not getattr(st, "name", None):
        return None
    try:
        idx = [list(st.name).index(n) for n in ARM_JOINT_NAMES]
    except ValueError:
        return None
    return np.array([st.position[i] for i in idx], dtype=np.float64)


def _sleep(client, seconds):
    """SIM-time sleep when the client exposes a sim clock, else wall time.

    Pacing the servo loop in sim time (one command per physics step) makes the motion
    speed/smoothness independent of the real-time factor, so a trajectory tuned at RTF~1
    (manipulation trainer) reproduces identically in the demo at RTF<1 (docker + full scene).
    Falls back to wall time if the client has no sim clock or /clock is not being published.
    """
    fn = getattr(client, "sleep", None)
    if callable(fn):
        fn(seconds)
    else:
        time.sleep(seconds)


# Hold detection - passive inner_finger_knuckle angle (deg). On an empty-hand close it reaches ~35deg;
# if an object blocks the fingers it stops below that (outer_knuckle is force-driven and pinned to the
# commanded angle -> no contact information). The driver publishes joint_states' inner_finger_knuckle (in rad).
INNER_FK_CLOSED_DEG = 35.0        # inner_finger_knuckle angle reached on empty-hand close (at grip_close~0.58)
INNER_FK_HOLD_MARGIN_DEG = 1.0    # if it falls short by this much (<34deg), an object is blocking = holding

# Waypoints where an object is held/moved with the gripper closed - hold is measured here.
#   lift  = whether it was grabbed (empty hand -> abort early as grasp failure)
#   over/place = whether it was dropped in transit (True->False means it fell during transport, abort)
_HOLD_CHECK_WPS = ("lift", "over", "place")


def grip_holding(client, grip_close, log, samples=6):
    """Determine whether an object is held from the inner_finger_knuckle (passive) angle.

    When commanded to close after grasp, an empty hand closes the fingers all the way and the
    inner_finger_knuckle reaches ~35deg; if an object spreads and blocks the fingers, it falls short and
    stops below that. Independent of object type/size (it can only reduce the closing, so <= 35). Uses
    the less-closed (smaller) side of the two fingers.
    (The grip_close argument is kept for backward compatibility and is not used in the decision.)
    Returns: True (holding) / False (empty hand) / None (no feedback).
    """
    _log = log if callable(log) else log.info
    thresh = np.deg2rad(INNER_FK_CLOSED_DEG - INNER_FK_HOLD_MARGIN_DEG)
    vals = []
    for _ in range(samples):
        try:
            st = client.get_arm_state()
        except Exception:
            st = None
        if st is not None and getattr(st, "name", None):
            fks = [float(p) for n, p in zip(st.name, st.position)
                   if "inner_finger_knuckle" in n]
            if fks:
                vals.append(min(fks))     # the less-closed (smaller) finger
        _sleep(client, 0.05)
    if not vals:
        _log("  [grip] no inner_finger_knuckle feedback - cannot determine holding")
        return None
    g = sum(vals) / len(vals)
    holding = bool(g < thresh)
    _log(f"  [grip] inner_fk={np.rad2deg(g):.1f}deg "
         f"(empty~{INNER_FK_CLOSED_DEG:.0f}, thresh<{INNER_FK_CLOSED_DEG-INNER_FK_HOLD_MARGIN_DEG:.0f}) "
         f"-> holding={holding}")
    return holding


def run_motion(client, log, p, phase="full"):
    """Publish pick-and-place via the closed-loop DLS servo (blocking).

    Returns: gripper hold decision (True/False/None) - computed from gripper feedback right after reaching lift.
    """
    _log = log if callable(log) else log.info
    kin = _kin()
    base_quat = arm_kin.R_to_quat(kin.fk(arm_kin.DEFAULT_ARM_Q)[:3, :3])

    # grasp_yaw: roll the gripper about the tool axis (EE local Z) - align the finger direction to the tumbler.
    # We roll the target orientation directly so the closed-loop DLS does not fight to undo the orientation
    # (position unchanged, only the wrist rotates). Maintained throughout.
    gy = float(p.get("grasp_yaw", 0.0))
    hold_quat = arm_kin.quat_mul(base_quat,
                                 arm_kin.rotvec_to_quat(np.array([0.0, 0.0, gy])))
    n = float(np.linalg.norm(hold_quat))
    if n > 0:
        hold_quat = hold_quat / n

    waypoints = build_waypoints(p, phase)  # (label, ee, grip, hold, mode)

    rate, dt = 20.0, 1.0 / 20.0
    # Scale the base cart step (0.006 m/tick @20Hz) by move_speed. 0.35 -> ~0.006.
    pos_step = max(0.004, float(p.get("move_speed", 0.35)) * 0.017)
    # joint-mode rotation speed (rad/tick) = carry_speed(rad/s) / rate.
    joint_step = max(0.003, float(p.get("carry_speed", 0.18)) / rate)

    seq = arm_kin.WaypointSequence(
        kin, arm_kin.DEFAULT_ARM_Q, hold_quat, pos_step, joint_step,
        _Log(log), waypoints,
    )

    # settle_start requires joint feedback, so wait until the first arm_q arrives.
    _log("  pick_and_place start (placo cart(DLS)/joint(arc) waypoint servo)")
    dbg = _dbg_open(phase, waypoints, pos_step, joint_step, rate)
    seq.start()
    grip_close = p.get("grip_close", GRIPPER_CLOSE)
    holding = None            # hold decision (True/False/None). The caller treats None as success.
    probed = set()            # hold-check waypoint labels already measured (prevents duplicate measurement)
    aborted = False           # whether the sequence was aborted early due to failure
    prev_q = None
    prev_t = time.monotonic()
    prev_label = None
    wp_t0 = prev_t
    tick_i = 0
    stale = 0
    no_fb = 0
    while seq.active:
        arm_q = _read_arm_q(client)
        if arm_q is None:
            no_fb += 1
            if dbg and no_fb % 20 == 1:
                dbg.write(f"  [wait] no arm_state x{no_fb}\n"); dbg.flush()
            _sleep(client, dt)
            continue
        cmd = seq.tick(arm_q)
        if cmd is not None:
            q, grip = cmd   # grip = gripper command value (0.0 open ~ grip_close)
            client.send_arm_command(_make_js(q, grip))
        if dbg is not None:
            now = time.monotonic()
            loop_dt = now - prev_t
            prev_t = now
            fresh = prev_q is None or not np.array_equal(arm_q, prev_q)
            if not fresh:
                stale += 1
            # act_dq = how much the actual joints moved since the last tick (rad). Repeated 0 -> stuck /
            # stale feedback; a large jump -> over-command / oscillation. cmd_dq = the command increment given this tick.
            act_dq = 0.0 if prev_q is None else float(np.linalg.norm(arm_q - prev_q))
            lbl, ph, idx = seq.label, seq._phase, seq._idx
            ee_err = float("nan")
            if ph in ("move", "dwell") and 0 <= idx < len(waypoints):
                ee = kin.fk(arm_q)[:3, 3]
                ee_err = float(np.linalg.norm(np.asarray(waypoints[idx][1], float) - ee))
            cmd_dq = (float(np.linalg.norm(np.asarray(cmd[0], float) - arm_q))
                      if cmd is not None else 0.0)
            if lbl != prev_label:
                if prev_label is not None:
                    dbg.write(f"  --- '{prev_label}' took {now - wp_t0:.2f}s ---\n")
                wp_t0 = now
                prev_label = lbl
            if tick_i % 5 == 0:
                dbg.write(f"  t={tick_i:4d} dt={loop_dt*1000:5.1f}ms {str(ph):11s} "
                          f"wp={str(lbl):9s} ee_err={ee_err:.4f} cmd_dq={cmd_dq:.4f} "
                          f"act_dq={act_dq:.4f} fresh={int(fresh)}\n")
                dbg.flush()
            prev_q = arm_q.copy()
            tick_i += 1
        # At each waypoint where an object is held/moved with the gripper closed (lift/over/place),
        # measure holding once. lift = did it grab (empty hand -> abort early as grasp failure),
        # over/place = was it dropped in transit (a prior True flipping to False means it fell, abort).
        lbl = seq.label
        if lbl in _HOLD_CHECK_WPS and lbl not in probed:
            probed.add(lbl)
            h = grip_holding(client, grip_close, _log)
            if h is None:
                pass                      # no feedback -> defer the decision (keep the prior value)
            elif lbl == "lift":
                holding = h
                if not h:                 # empty hand -> grasp failure
                    _log("  [pick] empty hand at lift (grasp failure) detected -> aborting sequence")
                    aborted = True
                    seq.abort()
            else:                         # over / place - in transit
                if holding and not h:     # was holding, then dropped -> fell
                    _log(f"  [pick] object dropped at {lbl} (transport failure) detected -> aborting sequence")
                    holding = False
                    aborted = True
                    seq.abort()
                elif h:
                    holding = True        # still confirmed holding
        _sleep(client, dt)

    # Recovery after an immediate abort - open the gripper, first ascend vertically, then return to the
    # default pose. Note: snapping the joints straight to the default pose from the grasp/lift pose makes
    # the folding path of the arm pass through and get stuck in the mobile robot's chassis. So we raise the
    # current EE vertically with a cart servo to leave the low workspace (arm extended upward), then command
    # the default pose.
    if aborted and arm_q is not None:
        try:
            cur_ee = kin.fk(arm_q)[:3, 3]
            ascend_z = float(p.get("carry_up", 0.246)) + 0.10   # safe ascent height (arm_base Z)
            tgt = np.array([cur_ee[0], cur_ee[1], max(float(cur_ee[2]), ascend_z)], float)
            tpos = np.array(cur_ee, float)
            for _ in range(int(2.5 * rate)):        # ascent servo (up to ~2.5s)
                aq = _read_arm_q(client)
                if aq is None:
                    _sleep(client, dt)
                    continue
                d = tgt - tpos
                dn = float(np.linalg.norm(d))
                tpos = tgt if dn <= pos_step else tpos + d / dn * pos_step
                q = arm_kin.dls_solve(kin, aq, tpos, hold_quat)
                client.send_arm_command(_make_js(q, GRIPPER_OPEN))
                _sleep(client, dt)
                if float(np.linalg.norm(kin.fk(aq)[:3, 3] - tgt)) < 0.02:
                    break
            client.send_arm_command(_make_js(arm_kin.DEFAULT_ARM_Q, GRIPPER_OPEN))
            _log("  aborted -> return to default pose after ascent (gripper open)")
        except Exception as e:
            _log(f"  aborted recovery failed ({e}) - keeping current pose")
            try:
                client.send_arm_command(_make_js(arm_q, GRIPPER_OPEN))
            except Exception:
                pass

    if dbg is not None:
        stale_pct = 100.0 * stale / max(1, tick_i)
        dbg.write(f"  === DONE ticks={tick_i} stale_feedback={stale_pct:.0f}% "
                  f"no_fb_waits={no_fb} holding={holding} aborted={aborted} ===\n")
        dbg.flush()
        dbg.close()
    _log(f"  {phase} DONE (holding={holding}{', ABORTED' if aborted else ''})")
    return holding


def pick_and_place(client, log, grasp_xy=None):
    """Run pick-and-place once.

    If grasp_xy=(fwd, lat) is given, override the xy of the grasp/approach point with those values (the
    object's actual arm_base-frame position). Descent depth, transport, basket, and orientation keep
    the default values.
    """
    p = load_params()
    if grasp_xy is not None:
        p["grasp_fwd"] = float(grasp_xy[0])
        p["grasp_lat"] = float(grasp_xy[1])
    return run_motion(client, log, p, "full")


# Backward-compatibility alias.
def run_pick_sequence(client, log, grasp_xy=None):
    return pick_and_place(client, log, grasp_xy)
