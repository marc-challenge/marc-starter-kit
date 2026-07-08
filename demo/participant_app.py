"""MARC 2026 SDK demo participant (baseline code).

The baseline demo that wires the whole flow together with **marc_sdk** --
Stage 1 grounding + Stage 2 occupancy A* driving (cmd_vel).
The protocol/ROS wiring (register, subscribe/publish, world_pose, occupancy) is all
handled by ``MARCClient``, so only the participant logic (mock VLA + nav) is connected
through handlers.

Run:
    source /opt/ros/humble/setup.bash
    export MARC_TEAM_ID=u1
    export MARC_TOKEN=<token>            # your assigned token (required; issued by the organizers)
    python3 participant_app.py
  or
    ./launch.sh u1

Precondition: the simulation platform must be running first.
"""

import logging
import math
import os
import time

from marc_sdk import MARCClient

from mock_agent_vla import MockVLAGrounding
from mock_agent_navigation import (
    OccupancyGridPlanner, PathFollower, GoalSeekController,
    load_obstacles, obstacles_excluding_goal, load_mock_data,
)
# Closed-loop placo pick from the manipulation kit (arm_pick.py): grasp_yaw wrist-roll +
# DLS servo, targeting the object position. Requires placo (installed in the demo image).
# The fixed-keyframe mock in mock_agent_manipulation.py remains as a dependency-free
# fallback reference.
from arm_pick import run_pick_sequence

log = logging.getLogger("participant_app")

# -- Stage 2 navigation parameters --
# Demo-internal giveup so the drive loop cannot spin forever if a goal is unreachable. This is NOT
# the competition timeout - the platform counts and enforces the real Stage 2 time limit.
STAGE2_NAV_TIMEOUT_S = 1800.0
STAGE2_NAV_RATE_HZ = 10.0
STAGE2_NAV_DT = 1.0 / STAGE2_NAV_RATE_HZ
STAGE2_INFLATION_M = 0.25      # erode free by the robot radius (avoids road<->grass step stalls)
STAGE2_STUCK_TIME_S = 8.0      # if movement < threshold for this long, replan from the current pose
STAGE2_STUCK_MOVE_M = 0.05
GOAL_TOLERANCE = 0.7           # reached test (m)

# -- Pick approach --
# Stop the chassis this far (m) in front of the object, then close the last stretch with a
# straight approach (the planner cannot enter a tight spot fully). Avoids driving over the object.
TUMBLER_STANDOFF_M = 0.58
# car_base(world_pose) -> arm_base forward offset. arm_base is ~0.18 m ahead of the chassis pose.
ARM_BASE_FWD = 0.18
# Grasp point relative to the object along +fwd. Negative = slightly past the object center
# so the gripper closes around its body (a small backoff grabbed only the edge -> dropped in
# transit), tuned slightly forward for a secure grip.
GRASP_FWD_BACKOFF = -0.01

# -- Stage 2 delivery goal --
# The Stage 2 task drives to the target object and then delivers it to the owner zone.
# The pickup goal (target position) comes from the reveal (msg 411) at runtime, with the
# Stage 2 grounding answer's target_coord as a fallback (see _wait_for_collect_goal). The
# delivery goal comes from owner_position in the mission (msg 211), with delivery_coord from
# mock_demo_data.yaml as a fallback. delivery_coord is pre-extracted from the scenario by
# tools/mock_data_builder/gen_mock_demo_data.py; real participants do not have the scenario.
_DELIVERY_COORD = load_mock_data().get("delivery_coord") or [-55.29138, 142.09763, 16.47326]
DELIVERY_XY = (_DELIVERY_COORD[0], _DELIVERY_COORD[1])


class DemoParticipant:
    """SDK-based demo participant -- connects the VLA/nav logic through handlers."""

    def __init__(self):
        self.client = MARCClient.from_env()
        self.vla = MockVLAGrounding()
        # Stage 2 pickup goal -- filled from the approximate location hint in the reveal (msg 411).
        # If not received, fall back to the Stage 2 answer target (_stage2_target_xy).
        self._collect_xy = None
        # Fallback pickup goal -- the Stage 2 grounding answer's target_coord (set in
        # _on_stage2_mission), used only if the reveal does not arrive.
        self._stage2_target_xy = None
        # Stage 2 delivery goal (owner) -- filled from owner_position in mission (msg 211).
        # If not included (older runtime), fall back to delivery_coord (DELIVERY_XY).
        self._pickup_xy = None
        self._occ_logged = False
        # Obstructions from the scenario yaml (landmarks such as vehicles) -- inject static
        # obstacles not in the occupancy map into the planner to route around them (demo only).
        self._obstacles = load_obstacles()
        self._register_handlers()

    # -- handler registration --

    def _register_handlers(self):
        c = self.client
        c.on_mission(self._on_mission)
        c.on_stage2_mission(self._on_stage2_mission)
        c.on_stage2_reveal(self._on_stage2_reveal)
        c.on_stage2_run(self._drive)
        c.on_score(self._on_score)
        c.on_state_change(self._on_state_change)
        c.on_time_expired(self._on_time_expired)

    # -- Stage 1 --

    def _on_mission(self, mission):
        log.info("[Stage1] round %s/%s: \"%s\"",
                 mission.round, mission.total_rounds, mission.voice_command)
        result = self.vla.process(mission.voice_command, self.client.list_cctv())
        # result is a GroundingResult -- submit it directly.
        self.client.submit_grounding(result)

    # -- Stage 2: submit interpretation -> drive --

    def _on_stage2_mission(self, mission):
        log.info("[Stage2] task: \"%s\" (limit %.0fs)",
                 mission.task_description, mission.time_limit)
        # Together with the VLA command, the platform provides the delivery goal (owner) coordinates (msg 211 owner_position).
        op = getattr(mission, "owner_position", None)
        if op and len(op) >= 2:
            self._pickup_xy = (float(op[0]), float(op[1]))
            log.info("[Stage2] owner_position(runtime-provided)=%s", self._pickup_xy)
        # Grounding: look up the pre-extracted Stage 2 answer for this task (mock VLA).
        result = self.vla.process_stage2(mission.task_description)
        self.client.submit_stage2_grounding(result)
        # Keep the answer's target as a fallback pickup goal (used if no reveal arrives).
        tc = result.target_coord
        if tc and len(tc) >= 2:
            self._stage2_target_xy = (float(tc[0]), float(tc[1]))
        log.info("[Stage2] submitted grounding target=%s, delivery point=%s",
                 self._stage2_target_xy, self._pickup_xy or DELIVERY_XY)

    def _on_stage2_reveal(self, reveal):
        """Stage 2 post-grounding reveal (msg 411) -- adopt the approximate location/type as the pick goal.

        Regardless of grounding accuracy, use the revealed approximate location
        (hint_center) as the goal of the pickup leg. There may be distractors within the
        radius, so use the type (target_type) to decide which object to pick up (the demo
        only demonstrates reaching the location).
        """
        log.info("[Stage2] reveal received -- score=%s, type=%s, center=%s, r=%.1f",
                 reveal.grounding_score, reveal.target_type,
                 reveal.hint_center, reveal.hint_radius)
        if reveal.hint_center and len(reveal.hint_center) >= 2:
            self._collect_xy = (float(reveal.hint_center[0]), float(reveal.hint_center[1]))

    def _drive(self):
        """Called once in a separate thread on entering STAGE2_RUN -- pick up the object then deliver it.

        Flow: stow the arm to lower the CoG -> approach the object standoff -> straight final
        approach into arm reach -> pick (read grip-hold) -> stow again if empty-handed ->
        deliver to the owner zone -> task_complete. All legs share one sim-time budget.
        """
        c = self.client
        # SIM-time deadline (via /clock) -- shared across all legs, robust to real-time factor < 1.
        deadline = c.now_s() + STAGE2_NAV_TIMEOUT_S

        # Poll briefly until the reveal (approximate location) arrives after scoring -> pickup goal.
        collect_xy = self._wait_for_collect_goal(timeout=5.0)
        # Delivery goal = runtime-provided owner_position (msg 211); if not included, fall back to delivery_coord.
        pickup_xy = self._pickup_xy if self._pickup_xy is not None else DELIVERY_XY
        log.info("[Stage2] navigation start -- collect=%s deliver=%s (timeout %.0fs)",
                 collect_xy, pickup_xy, STAGE2_NAV_TIMEOUT_S)

        # Top-heavy robot tip-over prevention: fold the arm to HOME before driving -> lower CoG.
        self._stow_arm()

        picked = False
        # -- leg 1: approach the object front standoff, then straight-approach into arm reach, then pick --
        approach = self._standoff_goal(collect_xy, TUMBLER_STANDOFF_M)
        log.info("[Stage2] leg1 -> object %s front standoff %s",
                 tuple(round(v, 2) for v in collect_xy), tuple(round(v, 2) for v in approach))
        # Aggressive planner tuning for the tight pick approach (low inflation + wide goal-exclusion).
        self._navigate_to(approach, deadline, tol=0.45, inflation_m=0.10, exclude_m=1.0)
        if not c.is_running:
            return
        # Final straight approach: drive straight at the object until arm_base fwd reaches grasp reach.
        self._approach_straight(collect_xy, deadline, target_fwd=0.40)
        # Stop! the chassis driver keeps applying the last cmd_vel, so emit 0 before the pick
        # (otherwise residual speed overshoots the object).
        for _ in range(5):
            c.send_cmd_vel(0.0, 0.0)
            c.sleep(0.1)
        log.info("[Stage2] standoff stop -- ready to pick")
        try:
            grasp_xy = self._object_arm_xy(collect_xy)   # actual object position in the arm_base frame
            log.info("[Stage2] object pick sequence (object-position grasp=%s)",
                     tuple(round(v, 3) for v in grasp_xy) if grasp_xy else None)
            held = run_pick_sequence(c, log, grasp_xy)
            c.send_cmd_vel(0.0, 0.0)
            # Grip-hold feedback (participant style). None (no feedback) -> conservatively count
            # the completed sequence as success.
            picked = True if held is None else bool(held)
            log.info("[Stage2] pick %s (grip held=%s)", "ok" if picked else "failed", held)
        except Exception as e:
            log.warning("[Stage2] pick failed: %s", e)

        # If the arm is left raised (front_high) without an object, driving the delivery leg with
        # that top-heavy CoG can tip the robot over on turns. Empty-handed -> fold to HOME.
        if not picked:
            self._stow_arm()

        # -- leg 2: deliver to the owner zone --
        log.info("[Stage2] leg2 -> deliver to owner zone %s", pickup_xy)
        reached = self._navigate_to(pickup_xy, deadline)

        c.stop()
        log.info("[Stage2] driving finished (delivery_reached=%s) -> task_complete", reached)
        c.task_complete()

    def _stow_arm(self):
        """Fold the arm to the HOME pose to lower the center of gravity -- tip-over prevention.

        The arm driver re-applies the last joint_command every tick, so one publish holds the pose
        for the whole leg. (Not called after a successful pick -- leg 2 keeps the carry pose.)
        """
        try:
            from arm_pick import HOME, GRIPPER_OPEN, _make_js
            self.client.send_arm_command(_make_js(HOME, GRIPPER_OPEN))
            log.info("[Stage2] arm stow -> HOME (lower CoG, tip-over prevention)")
        except Exception as e:
            log.warning("[Stage2] arm stow failed: %s", e)

    def _standoff_goal(self, dest, standoff):
        """A point ``standoff`` metres in front of ``dest`` along the robot->dest line.

        Approaching a standoff (instead of the object itself) keeps the chassis from driving
        onto/over the object.
        """
        pose = self.client.get_world_pose()
        if pose is None:
            return dest
        x, y, _ = pose
        dx, dy = dest[0] - x, dest[1] - y
        d = math.hypot(dx, dy)
        if d <= standoff:
            return (x, y)
        f = (d - standoff) / d
        return (x + dx * f, y + dy * f)

    def _object_arm_xy(self, obj_xy):
        """Object world XY -> arm_base frame (fwd, lat) using the current robot pose.

        Rotate the world offset into the car_base frame, then shift by the
        car_base->arm_base forward offset. Lets the closed-loop pick aim at the real
        object regardless of small nav misalignment.
        """
        pose = self.client.get_world_pose()
        if pose is None:
            return None
        x, y, yaw = pose
        dx, dy = obj_xy[0] - x, obj_xy[1] - y
        cc, ss = math.cos(yaw), math.sin(yaw)
        fwd = dx * cc + dy * ss            # car_base forward (+X)
        lat = -dx * ss + dy * cc           # car_base left (+Y)
        return (fwd - ARM_BASE_FWD - GRASP_FWD_BACKOFF, lat)

    def _approach_straight(self, obj_xy, deadline, target_fwd=0.40, max_t=20.0):
        """Drive straight at the object until the arm_base forward distance reaches target_fwd.

        The planner cannot enter a tight spot fully (it halts ~1 m out), so fill the remaining
        empty space by driving straight into the object, with a small lateral steering term to
        center it (lat -> 0). SIM-time paced.
        """
        c = self.client
        t_end = min(deadline, c.now_s() + max_t)
        while c.is_running and c.now_s() < t_end:
            gxy = self._object_arm_xy(obj_xy)
            if gxy is None:
                break
            fwd, lat = gxy
            if fwd <= target_fwd + 0.02:
                break
            ang = max(-0.4, min(0.4, 1.5 * lat))         # center the object (lat -> 0)
            lin = 0.12 if abs(lat) < 0.15 else 0.06      # badly off -> creep slower while turning
            c.send_cmd_vel(linear_x=lin, angular_z=ang)
            c.sleep(STAGE2_NAV_DT)
        for _ in range(5):
            c.send_cmd_vel(0.0, 0.0)
            c.sleep(0.05)
        gxy = self._object_arm_xy(obj_xy)
        log.info("[Stage2] straight approach done -- object arm coords %s",
                 tuple(round(v, 3) for v in gxy) if gxy else None)

    def _wait_for_collect_goal(self, timeout: float = 5.0):
        """Poll briefly until the reveal (msg 411) approximate position arrives, then return the pickup goal (x, y).

        When the reveal arrives, _on_stage2_reveal fills self._collect_xy. If not received
        within the timeout, fall back to the Stage 2 grounding answer's target (_stage2_target_xy).
        """
        c = self.client
        deadline = c.now_s() + timeout
        while c.now_s() < deadline:
            if self._collect_xy is not None:
                return self._collect_xy
            c.sleep(0.1)
        log.warning("[Stage2] reveal not received (%.0fs) -- falling back to grounding target %s",
                    timeout, self._stage2_target_xy)
        return self._stage2_target_xy

    def _navigate_to(self, goal, deadline, tol=GOAL_TOLERANCE,
                     inflation_m=STAGE2_INFLATION_M, exclude_m=0.6, cruise=False):
        """Drive to a single goal (x, y) by following occupancy A*. True if reached, False on timeout.

        Args:
            tol: reached-test XY distance (m). Tighter for a tight pick approach.
            inflation_m: erode the free area by this much (lower = enter tighter spots).
            exclude_m: drop scenario obstacles within this radius of the goal (keep it reachable).
            cruise: True -> pass-through waypoint, no near-goal deceleration.
        The deadline is a shared SIM-time budget across all Stage 2 legs.
        """
        c = self.client
        planner = OccupancyGridPlanner()
        follower = None
        fallback_ctrl = GoalSeekController()

        # Obstacles for this leg -- exclude the goal itself (pickup target/owner) and avoid only the surrounding obstructions.
        leg_obstacles = obstacles_excluding_goal(self._obstacles, goal, exclude_m=exclude_m)

        last_xy = None
        last_progress_t = c.now_s()
        last_log_t = 0.0
        reached_goal = False

        while c.is_running and c.now_s() < deadline:
            pose = c.get_world_pose()
            if pose is None:
                c.sleep(STAGE2_NAV_DT)
                continue
            x, y, yaw = pose
            dist = math.hypot(goal[0] - x, goal[1] - y)
            if dist < tol:
                reached_goal = True
                log.info("[STAGE2] reached goal at (%.2f, %.2f)", x, y)
                break

            # Refresh the occupancy grid (from the occupancy map)
            occ = c.get_occupancy_map()
            if occ is not None:
                planner.update_from_msg(occ)
                if not self._occ_logged:
                    self._occ_logged = True
                    log.info("[Stage2] occupancy received: %dx%d res=%.3f origin=(%.1f, %.1f)",
                             occ.info.width, occ.info.height, occ.info.resolution,
                             occ.info.origin.position.x, occ.info.origin.position.y)

            # Detect stall -> trigger replanning
            need_replan = follower is None
            if last_xy is not None:
                if math.hypot(x - last_xy[0], y - last_xy[1]) > STAGE2_STUCK_MOVE_M:
                    last_progress_t = c.now_s()
                elif c.now_s() - last_progress_t > STAGE2_STUCK_TIME_S:
                    log.info("[Stage2] stall detected -> replanning")
                    need_replan = True
                    last_progress_t = c.now_s()
            last_xy = (x, y)

            if need_replan:
                if not planner.ready:
                    if not self._occ_logged:
                        log.warning("[Stage2] occupancy not received -> straight-line fallback")
                else:
                    # Increase the snap radius so start/goal off the road (free) are also corrected to nearby free.
                    # Inject scenario obstructions (vehicles/bicycles/people/objects) as obstacles to route around.
                    path = planner.plan((x, y), goal, inflation_m=inflation_m,
                                        snap_m=8.0, extra_obstacles=leg_obstacles)
                    if path:
                        follower = PathFollower(path, passthrough=cruise)
                        log.info("[Stage2] occupancy A* path %d waypoints", len(path))
                    else:
                        sr, sc = planner._w2c(x, y)
                        gr, gc = planner._w2c(*goal)
                        sv = int(planner._grid[sr, sc]) if planner._in_bounds(sr, sc) else 99
                        gv = int(planner._grid[gr, gc]) if planner._in_bounds(gr, gc) else 99
                        log.warning("[Stage2] A* failed -> straight-line fallback "
                                    "(start cell=(%d,%d) val=%d, goal cell=(%d,%d) val=%d) "
                                    "[0=free 100=occ -1=unknown]", sr, sc, sv, gr, gc, gv)

            # Compute commands: follow the path if there is one, otherwise straight-line fallback
            wp_str = ""
            if follower is not None:
                linear, angular, done, info = follower.compute(pose)
                wp_str = " wp %d/%d=(%.1f, %.1f)" % (
                    info["idx"], info["total"], info["waypoint"][0], info["waypoint"][1])
                if done:
                    reached_goal = True
                    log.info("[STAGE2] reached goal at (%.2f, %.2f)", x, y)
                    break
            else:
                linear, angular, reached = fallback_ctrl.compute(pose, goal, cruise=cruise)
                if reached:
                    reached_goal = True
                    log.info("[STAGE2] reached goal at (%.2f, %.2f)", x, y)
                    break

            c.send_cmd_vel(linear_x=linear, angular_z=angular)

            # Progress log (throttled to ~1s SIM time) -- for tracking pose/dist/v/w
            now = c.now_s()
            if now - last_log_t >= 1.0:
                log.info("[STAGE2] pose=(%.2f, %.2f, yaw=%.0fdeg) dist=%.2fm%s v=%.2f w=%+.2f",
                         x, y, math.degrees(yaw), dist, wp_str, linear, angular)
                last_log_t = now

            c.sleep(STAGE2_NAV_DT)

        return reached_goal

    # -- notifications --

    def _on_score(self, score):
        if score.is_final:
            log.info("[FINAL] %s", score.scores)
        else:
            log.info("[Score] round %s: total=%s", score.round, score.total)

    def _on_state_change(self, old, new):
        log.info("[STATE] %s -> %s", old, new)

    def _on_time_expired(self, which):
        log.info("[TIME] expired: %s", which)

    # -- run --

    def run(self):
        if not self.client.connect():
            raise SystemExit("Registration failed -- check the runtime and the token.")
        self.client.run()


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] %(name)s %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    DemoParticipant().run()


if __name__ == "__main__":
    main()
