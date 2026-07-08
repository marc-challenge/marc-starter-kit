"""6-DoF arm kinematics engine for the pick-and-place reference code (placo based).

Uses placo (Pinocchio) to compute the URDF FK/Jacobian; every tick it reads the arm joint_states
and, with a single-step DLS (damped-least-squares) closed-loop servo, drives the end-effector to
converge on the target waypoint.

Components:
    PlacoArmKinematics - loads the URDF, FK(4x4) + 6x6 arm Jacobian(local_world_aligned)
    dls_solve          - single-step DLS IK step (lambda=0.05)
    ik_solve           - iterates dls_solve until convergence (computes joint-mode target angles)
    WaypointSequence   - waypoint state machine. On top of the closed-loop servo (cart), it supports
                         the per-waypoint format (label, ee, grip, hold, mode).
                         mode "cart" = DLS straight-line EE tracking, "joint" = joint-space interpolation arc.
"""

from __future__ import annotations

import time

import numpy as np
import placo


# Default arm-joint pose that preserves the EE orientation.
DEFAULT_ARM_Q = np.array([0.0, 0.262, -2.094, 0.0, -0.785, 1.571])

# DLS damping coefficient.
LAMBDA_DLS = 0.05

# State-machine thresholds.
POS_TOL = 0.02      # m   - EE must be within this of a waypoint to advance
GRIP_DWELL = 1.0    # s   - pause at a waypoint while the gripper opens/closes
WP_TIMEOUT = 15.0   # s   - give up on a waypoint the IK cannot reach
SETTLE_TOL = 0.05   # rad - joint error that counts as "arm settled at pose"
# feedback-paced leash: in cart mode, keep the "move target (target_pos)" from getting more
# than this distance ahead of the actual EE. In a slow sim where joint_states arrive slower than
# the command rate, the target waits for the arm and advances only as much as the arm has caught
# up -> the DLS step always stays small so the path stays straight (independent of the real-time
# factor). On a fast machine the lead is about pos_step, so it advances every tick = same as before.
CART_LEASH = 0.03   # m


# ----- math helpers (wxyz quaternion) -----

def quat_mul(a, b):
    aw, ax, ay, az = a; bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ])


def quat_error_axis_angle(q_target, q_current):
    qc_inv = np.array([q_current[0], -q_current[1], -q_current[2], -q_current[3]])
    q_err = quat_mul(q_target, qc_inv)
    if q_err[0] < 0:
        q_err = -q_err
    w = float(np.clip(q_err[0], -1.0, 1.0))
    s = float(np.sqrt(max(0.0, 1.0 - w*w)))
    if s < 1e-8:
        return np.zeros(3)
    angle = 2.0 * np.arccos(w)
    return q_err[1:4] / s * angle


def rotvec_to_quat(rv):
    a = float(np.linalg.norm(rv))
    if a < 1e-8:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = rv / a
    s = np.sin(a/2)
    return np.array([np.cos(a/2), axis[0]*s, axis[1]*s, axis[2]*s])


def R_to_quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> wxyz quaternion."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0.0:
        s = 0.5 / np.sqrt(t + 1.0)
        return np.array([0.25 / s,
                         (R[2, 1] - R[1, 2]) * s,
                         (R[0, 2] - R[2, 0]) * s,
                         (R[1, 0] - R[0, 1]) * s])
    if R[0, 0] >= R[1, 1] and R[0, 0] >= R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        return np.array([(R[2, 1] - R[1, 2]) / s,
                         0.25 * s,
                         (R[0, 1] + R[1, 0]) / s,
                         (R[0, 2] + R[2, 0]) / s])
    if R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        return np.array([(R[0, 2] - R[2, 0]) / s,
                         (R[0, 1] + R[1, 0]) / s,
                         0.25 * s,
                         (R[1, 2] + R[2, 1]) / s])
    s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
    return np.array([(R[1, 0] - R[0, 1]) / s,
                     (R[0, 2] + R[2, 0]) / s,
                     (R[1, 2] + R[2, 1]) / s,
                     0.25 * s])


# ----- placo wrapper: FK + arm Jacobian -----

class PlacoArmKinematics:
    """Loads a URDF via placo and exposes FK + 6x6 arm-only Jacobian."""

    def __init__(self, urdf_path: str, target_frame: str, joint_names: list[str]):
        self._robot = placo.RobotWrapper(urdf_path)
        self._target = target_frame
        self._joints = list(joint_names)
        # Column indices into the 6×nv Jacobian for our 6 arm joints (skipping
        # the 6 floating-base columns and the gripper / mimic joints).
        self._cols = [self._robot.get_joint_v_offset(n) for n in self._joints]
        # (offset, size) for each joint in the q state vector. Continuous
        # (revolute_unbounded) joints have size=2 stored as (cos, sin); plain
        # revolute joints have size=1 storing the angle directly. placo's
        # set_joint writes only q[off] and corrupts the cos/sin pair, so we
        # manipulate state.q ourselves.
        self._q_layout = [(self._robot.get_joint_offset(n),
                           self._robot.get_joint_size(n)) for n in self._joints]

    def _set_q(self, q_rad: np.ndarray) -> None:
        q = self._robot.state.q.copy()
        for (off, size), val in zip(self._q_layout, q_rad):
            if size == 1:
                q[off] = float(val)
            elif size == 2:
                q[off]     = float(np.cos(val))
                q[off + 1] = float(np.sin(val))
            else:
                raise ValueError(f"unexpected joint q_size={size}")
        self._robot.state.q = q
        self._robot.update_kinematics()

    def fk(self, q_rad: np.ndarray) -> np.ndarray:
        """4x4 transform of target frame in base_link (== world for fixed base)."""
        self._set_q(q_rad)
        return self._robot.get_T_world_frame(self._target)

    def jacobian(self, q_rad: np.ndarray) -> np.ndarray:
        """6x6 Jacobian for the 6 arm joints, in local_world_aligned frame.

        Rows are [linear_xyz, angular_xyz] expressed at the target frame
        origin with axes aligned to world (== base_link).
        """
        self._set_q(q_rad)
        J = self._robot.frame_jacobian(self._target, "local_world_aligned")
        return J[:, self._cols]


def dls_solve(kin: PlacoArmKinematics, arm_q: np.ndarray, target_pos: np.ndarray,
              target_quat: np.ndarray, lam: float = LAMBDA_DLS) -> np.ndarray:
    """Damped-least-squares IK step.

    dq = J^T (JJ^T + λ²I)^-1 e.
    """
    T = kin.fk(arm_q)
    pos_err = target_pos - T[:3, 3]
    rot_err = quat_error_axis_angle(target_quat, R_to_quat(T[:3, :3]))
    twist = np.concatenate([pos_err, rot_err])
    J = kin.jacobian(arm_q)
    damped = J @ J.T + (lam ** 2) * np.eye(6)
    return arm_q + J.T @ np.linalg.solve(damped, twist)


def ik_solve(kin: PlacoArmKinematics, target_pos: np.ndarray, target_quat: np.ndarray,
             q0: np.ndarray, iters: int = 200, lam: float = LAMBDA_DLS) -> np.ndarray:
    """Iterate the dls_solve step until convergence to get the joint angles of the target pose (for joint-mode arcs).

    Used to pre-solve the target joint angles for joint-space interpolation (unlike the cart servo,
    the joint angles are fixed first to enable a singularity-avoiding base-rotation arc).
    """
    q = np.array(q0, dtype=np.float64)
    for _ in range(iters):
        T = kin.fk(q)
        pe = target_pos - T[:3, 3]
        re = quat_error_axis_angle(target_quat, R_to_quat(T[:3, :3]))
        if np.linalg.norm(pe) < 1e-4 and np.linalg.norm(re) < 1e-3:
            break
        q = dls_solve(kin, q, target_pos, target_quat, lam)
    return q


def unwrap(q: np.ndarray, ref: np.ndarray) -> np.ndarray:
    """Offset each joint of q by +/-2*pi so it is closest to ref (shortest angular path during joint interpolation)."""
    q = np.asarray(q, dtype=np.float64).copy()
    for i in range(len(q)):
        q[i] += 2 * np.pi * round((ref[i] - q[i]) / (2 * np.pi))
    return q


# ----- waypoint sequence state machine (closed-loop servo + cart/joint waypoint format) -----

class WaypointSequence:
    """State machine that traverses a labeled waypoint list once, on demand.

    Each waypoint uses the format ``(label, ee_pos[arm_base], grip, hold_sec, mode)``:
        label - name for progress display / hold detection
        ee    - EE target position in the arm_base frame (orientation stays at default_quat)
        grip  - gripper command value (0.0 open ~ grip_close). latched when the waypoint is reached
        hold  - wait after reaching (seconds). waits for gripper open/close / drive convergence
        mode  - "cart": DLS straight-line EE tracking (closed-loop, vertically precise).
                "joint": solve target joint angles via IK and do joint-space interpolation
                         (base-rotation arc, singularity avoidance - prevents dropping during transport).

    Phases:
        ``settle_start`` - command the default pose, wait for the arm to settle
        ``move``         - approach the waypoint via cart (straight-line tracking) or joint (joint interpolation)
        ``dwell``        - after reaching, wait for hold seconds (gripper latch)
        ``settle_end``   - command the default pose again, wait, then go idle
    """

    def __init__(self, kin, default_arm_q, default_quat, pos_step, joint_step,
                 logger, waypoints):
        self._kin = kin
        self._default_arm_q = np.asarray(default_arm_q, dtype=np.float64)
        self._default_quat = np.asarray(default_quat, dtype=np.float64)
        self._pos_step = float(pos_step)      # m/tick (cart)
        self._joint_step = float(joint_step)  # rad/tick (joint)
        self._log = logger
        # normalize: (label, ee_array, grip_val, hold_sec, mode)
        self._waypoints = []
        for wp in waypoints:
            label, ee, grip, hold, mode = wp
            self._waypoints.append(
                (str(label), np.asarray(ee, dtype=np.float64),
                 float(grip), float(hold), str(mode)))

        self._phase: str | None = None
        self._idx = 0
        self._grip = 0.0            # current gripper command value
        self._target_pos = np.zeros(3)   # cart move target (for straight-line interpolation)
        self._joint_target = None        # joint-mode target joint angles (IK on entry)
        self._joint_cmd = None           # joint-mode current commanded joint angles
        self._retrace_q = None           # start configuration of the outgoing joint arc (for reverse retrace on return)
        self._dwell_until = 0.0
        self._wp_deadline = 0.0

    @property
    def active(self) -> bool:
        """True while the sequence owns the arm."""
        return self._phase is not None

    @property
    def label(self) -> str:
        """Current waypoint label ('' when idle/bracketing)."""
        if self._phase in (None, "settle_start", "settle_end"):
            return ""
        return self._waypoints[self._idx][0]

    def start(self) -> None:
        """Begin the sequence (no-op if it is already running)."""
        if self._phase is not None:
            return
        self._phase = "settle_start"
        self._idx = 0
        self._log.info("pick sequence requested — moving to default pose")

    def abort(self) -> None:
        """Stop the sequence immediately."""
        if self._phase is not None:
            self._phase = None
            self._log.info("pick sequence aborted")

    def tick(self, arm_q: np.ndarray):
        """Advance the state machine one control cycle.

        Returns ``(arm_q_command, grip_value)`` to publish, or ``None`` when
        there is nothing to send this cycle (the sequence just finished).
        """
        if self._phase is None:
            return None
        now = time.monotonic()

        # bracket phases: command the default pose, wait for the arm to settle
        if self._phase in ("settle_start", "settle_end"):
            settled = float(np.abs(arm_q - self._default_arm_q).max()) < SETTLE_TOL
            if not settled:
                return self._default_arm_q, self._grip
            if self._phase == "settle_start":
                self._idx = 0
                self._enter_move(arm_q, now)
                self._log.info("at default pose — traversing waypoints")
                return self._default_arm_q, self._grip
            self._phase = None
            self._log.info("pick sequence complete")
            return None

        if self._phase == "dwell":
            if now >= self._dwell_until:
                self._advance(arm_q, now)
            # During dwell, hold the last command (target angles for joint, the target-position solution for cart).
            if self._joint_cmd is not None:
                return self._joint_cmd, self._grip
            new_q = dls_solve(self._kin, arm_q, self._target_pos, self._default_quat)
            return new_q, self._grip

        # move phase
        label, wp_pos, wp_grip, wp_hold, mode = self._waypoints[self._idx]

        if mode == "joint":
            # Joint-space interpolation: monotonically move the current commanded angles toward the
            # target angles by joint_step each tick. It deterministically reaches the target, so there
            # is no timeout-snap (to keep the arm from jerking mid-motion).
            delta = self._joint_target - self._joint_cmd
            step = np.clip(delta, -self._joint_step, self._joint_step)
            self._joint_cmd = self._joint_cmd + step
            if float(np.max(np.abs(self._joint_target - self._joint_cmd))) < 1e-3:
                self._reach(now)
            return self._joint_cmd, self._grip

        # cart mode - DLS straight-line tracking
        timed_out = now >= self._wp_deadline
        ee_pos = self._kin.fk(arm_q)[:3, 3]
        to_wp = wp_pos - self._target_pos
        dist = float(np.linalg.norm(to_wp))
        # feedback-paced leash: if the move target got more than CART_LEASH ahead of the actual EE
        # (= the arm has not caught up yet, stale feedback in a slow sim), stop advancing and wait for
        # the arm to catch up. Once it does (or on a fast machine) the lead shrinks and it advances
        # again. The reached/timeout logic is unchanged, so there is no deadlock.
        lead = float(np.linalg.norm(self._target_pos - ee_pos))
        if lead < CART_LEASH:
            if dist > self._pos_step:
                self._target_pos = self._target_pos + to_wp / dist * self._pos_step
            else:
                self._target_pos = wp_pos.copy()
        ee_err = float(np.linalg.norm(wp_pos - ee_pos))
        reached = dist <= self._pos_step and ee_err < POS_TOL
        if timed_out and not reached:
            self._log.warn(f"waypoint {self._idx + 1}({label}) unreached "
                           f"(ee_err={ee_err:.3f} m) - advancing anyway")
            self._target_pos = wp_pos.copy()
        if reached or timed_out:
            self._reach(now)
        new_q = dls_solve(self._kin, arm_q, self._target_pos, self._default_quat)
        return new_q, self._grip

    def _enter_move(self, arm_q: np.ndarray, now: float) -> None:
        """Start the move to waypoint idx - initialize the target per mode."""
        self._phase = "move"
        self._wp_deadline = now + WP_TIMEOUT
        label, wp_pos, wp_grip, wp_hold, mode = self._waypoints[self._idx]
        if mode == "joint":
            self._joint_cmd = np.asarray(arm_q, dtype=np.float64).copy()
            # The outgoing (transport) arc and the return arc must rotate in exactly opposite
            # directions so the wrist does not wind up (CW to the basket -> CCW on return). If the
            # gripper is closed (= carrying an object), this is the outgoing arc: solve the target
            # angles via IK and remember the start configuration. If open (= returning), retrace that
            # remembered start configuration exactly (in reverse) to unwind.
            if self._grip > 1e-3 or self._retrace_q is None:
                # Outgoing arc: seed joint_1 toward the target's horizontal direction -> base arc, shortest unwrap.
                seed = self._default_arm_q.copy()
                seed[0] = float(np.arctan2(wp_pos[1], wp_pos[0]))
                tq = ik_solve(self._kin, wp_pos, self._default_quat, seed)
                self._joint_target = unwrap(tq, arm_q)
                self._retrace_q = self._joint_cmd.copy()   # start configuration to retrace on return
            else:
                # Return arc: retrace the outgoing start configuration in reverse (prevents twist/180deg).
                self._joint_target = self._retrace_q.copy()
                self._retrace_q = None
        else:
            self._joint_target = None
            self._joint_cmd = None
            self._target_pos = self._kin.fk(arm_q)[:3, 3].copy()

    def _reach(self, now: float) -> None:
        """Waypoint reached - latch the gripper, then dwell for hold seconds."""
        label, wp_pos, wp_grip, wp_hold, mode = self._waypoints[self._idx]
        self._grip = wp_grip
        self._phase = "dwell"
        self._dwell_until = now + max(0.0, wp_hold)
        self._log.info(
            f"waypoint {self._idx + 1}({label}) reached — grip={wp_grip:.3f} "
            f"hold={wp_hold:.1f}s")

    def _advance(self, arm_q: np.ndarray, now: float) -> None:
        self._idx += 1
        if self._idx >= len(self._waypoints):
            self._phase = "settle_end"
            self._log.info("all waypoints done — returning to default pose")
            return
        self._enter_move(arm_q, now)
        label = self._waypoints[self._idx][0]
        self._log.info(
            f"moving to waypoint {self._idx + 1}/{len(self._waypoints)} ({label})")
