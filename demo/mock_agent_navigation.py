"""Mock navigation agent -- stands in for the participant's Stage 2 navigation.

This module bundles the demo's navigation pieces (participants replace/extend them):
  - GoalSeekController / PathFollower -- P-control that returns ``(linear_x, angular_z)``
    scalars to publish with ``client.send_cmd_vel(...)``.
  - OccupancyGridPlanner -- A* path planning over the runtime occupancy grid
    (``MARCClient.get_occupancy_map()``).
  - load_obstacles / obstacles_excluding_goal -- extra 3D obstructions the occupancy map
    omits (vehicles/bicycles/people/objects), read from ``mock_demo_data.yaml``
    (pre-extracted by ``tools/mock_data_builder/gen_mock_demo_data.py``).
"""

import math


def _wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class GoalSeekController:
    """Simple proportional (P) goal-seeking controller.

    ``compute(pose, goal, cruise)`` returns one step of ``(linear_x, angular_z, reached)``.
    When the heading error is large it rotates in place; when it is small it moves forward
    with heading correction, decelerating near the goal (unless ``cruise``).
    """

    # Reached test -- XY distance (m)
    GOAL_TOLERANCE = 0.7
    # Above this heading error, rotate in place (linear=0 -> pure wz -> chassis X-pattern
    # in-place spin); below it, move forward with heading correction. The chassis can spin in
    # place (X-pattern steering + opposite left/right wheels), so large turns (e.g. a goal
    # behind the robot) spin in place first, then drive forward -- avoids the Ackermann-only
    # "turn around and stall" problem.
    HEADING_TOLERANCE = math.pi / 4   # ~45 deg: beyond this, rotate in place then advance

    # Speed / turn caps. Large turns are handled in place (see HEADING_TOLERANCE), so forward
    # motion rarely needs a hard swerve; the Ackermann w-limit below guarantees a tight arc.
    MAX_LINEAR = 0.8           # m/s
    MAX_ANGULAR = 1.2          # rad/s
    K_LINEAR = 0.5
    K_ANGULAR = 0.8
    # 4-wheel-steer track (m). The chassis stops (0) when 2*v <= w*T (a turn too tight for the
    # speed), so the controller caps w within 2v/T to always emit a valid (forward + steer)
    # command. Same value as the platform's chassis steering track.
    STEERING_TRACK = 0.38

    def compute(self, pose: tuple, goal: tuple, cruise: bool = False) -> tuple:
        """Compute one step of commands from the current pose and goal.

        Args:
            pose: (x, y, yaw_radians) -- world coordinates (client.get_world_pose()).
            goal: (gx, gy)           -- target XY coordinates (m).
            cruise: if True, skip the near-goal proportional deceleration (K_LINEAR*dist)
                and cruise at MAX_LINEAR -- for intermediate pass-through waypoints; only
                the final goal (cruise=False) decelerates on approach.

        Returns:
            (linear_x, angular_z, reached: bool)
        """
        x, y, yaw = pose
        gx, gy = goal
        dx = gx - x
        dy = gy - y
        dist = math.hypot(dx, dy)

        if dist < self.GOAL_TOLERANCE:
            return 0.0, 0.0, True

        target_yaw = math.atan2(dy, dx)
        heading_err = _wrap_pi(target_yaw - yaw)

        angular = _clamp(self.K_ANGULAR * heading_err,
                         -self.MAX_ANGULAR, self.MAX_ANGULAR)
        if abs(heading_err) > self.HEADING_TOLERANCE:
            linear = 0.0                       # rotate in place
        elif cruise:
            linear = self.MAX_LINEAR           # pass-through waypoint: no deceleration
        else:
            linear = _clamp(self.K_LINEAR * dist, 0.0, self.MAX_LINEAR)  # decelerate near goal

        # Ackermann validity: a turn too tight for the speed (2v <= w*T) makes the chassis
        # stop. Cap w within 2v/T (safety 0.9) so even a goal behind the robot keeps moving
        # in the tightest valid arc (this chassis must move forward to steer).
        if linear > 1e-3:
            w_max = 1.8 * linear / self.STEERING_TRACK
            angular = _clamp(angular, -w_max, w_max)

        return linear, angular, False


class PathFollower:
    """Controller that follows a waypoint path in sequence.

    Takes the world waypoint list produced by ``OccupancyGridPlanner.plan()`` and,
    every step, computes ``GoalSeekController`` commands from the current pose toward
    the active waypoint. When it gets close enough to the active waypoint it advances
    to the next, and it is done once it reaches the last waypoint (= goal).
    """

    # Distance to consider an intermediate waypoint passed (m). Looser than the goal
    # tolerance to smooth out corners.
    WAYPOINT_TOLERANCE = 1.0

    def __init__(self, path, controller: GoalSeekController = None,
                 passthrough: bool = False):
        self.path = [(float(x), float(y)) for (x, y) in path]
        self.idx = 0
        self.ctrl = controller or GoalSeekController()
        # passthrough=True: the final point is also a pass-through (no deceleration) --
        # used when this goal is itself a waypoint, not a stop point.
        self._passthrough = bool(passthrough)

    def compute(self, pose: tuple):
        """Compute one step of commands from the current pose.

        Returns:
            (linear_x, angular_z, done: bool, info: dict)
            info = {"idx", "total", "waypoint": (x, y)}
        """
        x, y, _ = pose
        # Skip intermediate waypoints that have been reached (keep the last one).
        while self.idx < len(self.path) - 1:
            wx, wy = self.path[self.idx]
            if math.hypot(wx - x, wy - y) < self.WAYPOINT_TOLERANCE:
                self.idx += 1
            else:
                break

        wx, wy = self.path[self.idx]
        # Intermediate waypoints cruise (full speed); the final point decelerates -- unless
        # passthrough, where the final point also cruises.
        final = self.idx >= len(self.path) - 1
        cruise = (not final) or self._passthrough
        linear, angular, reached = self.ctrl.compute(pose, (wx, wy), cruise=cruise)
        done = final and reached
        info = {"idx": self.idx, "total": len(self.path), "waypoint": (wx, wy)}
        return linear, angular, done, info


# ---------------------------------------------------------------------------
# Occupancy-grid A* path planning.
# ---------------------------------------------------------------------------
import heapq
import math

import numpy as np

SQRT2 = math.sqrt(2.0)


class OccupancyGridPlanner:
    """A* planner over an occupancy grid.

    Update the grid with ``update_from_msg()`` and get a list of world-coordinate
    waypoints with ``plan()``. If there is no grid, ``ready == False``.
    """

    def __init__(self):
        self._grid = None        # int8 (H, W), row0 = bottom
        self._res = None         # m/cell
        self._ox = None          # origin x (world)
        self._oy = None          # origin y (world)
        self._h = 0
        self._w = 0

    # -- grid update --

    def update_from_msg(self, msg) -> bool:
        """Update the grid from a nav_msgs/OccupancyGrid message."""
        info = msg.info
        w = int(info.width)
        h = int(info.height)
        if w == 0 or h == 0:
            return False
        grid = np.array(msg.data, dtype=np.int8).reshape(h, w)
        self._grid = grid
        self._res = float(info.resolution)
        self._ox = float(info.origin.position.x)
        self._oy = float(info.origin.position.y)
        self._h = h
        self._w = w
        return True

    @property
    def ready(self) -> bool:
        return self._grid is not None

    @property
    def resolution(self):
        return self._res

    # -- coordinate conversion --

    def _w2c(self, wx, wy):
        """world (x, y) -> cell (row, col)."""
        col = int((wx - self._ox) / self._res)
        row = int((wy - self._oy) / self._res)
        return row, col

    def _c2w(self, row, col):
        """cell (row, col) -> world (x, y) (cell center)."""
        wx = self._ox + (col + 0.5) * self._res
        wy = self._oy + (row + 0.5) * self._res
        return wx, wy

    # -- path planning --

    def plan(self, start_xy, goal_xy, inflation_m=0.25, margin_m=12.0,
             snap_m=1.5, extra_obstacles=None):
        """Return a detour path from start_xy -> goal_xy as a list of world waypoints.

        Args:
            start_xy: (x, y) current robot position (world).
            goal_xy:  (x, y) target position (world).
            inflation_m: erode the free area by the robot radius to build a path that
                         stays away from the road boundary (avoids step stalls).
            margin_m: search margin outside the start/goal bounding box (crop for planning speed).
            snap_m:  maximum radius to correct a start/goal cell to the nearest free
                     cell when it is not traversable.
            extra_obstacles: [(x, y, radius_m), ...] static obstacles not in the
                     occupancy map (vehicles etc.). Their disc is carved out as not traversable.

        Returns:
            [(x, y), ...] path waypoints. If the goal cell is traversable the last one is
            exactly goal_xy; if it is blocked (a vehicle etc.), up to the free cell nearest
            the goal. None on failure.
        """
        if not self.ready:
            return None

        sr, sc = self._w2c(*start_xy)
        gr, gc = self._w2c(*goal_xy)
        if not (self._in_bounds(sr, sc) and self._in_bounds(gr, gc)):
            return None

        # 1) Crop only around start/goal -- avoid A*/erosion over the full grid (tens of millions of cells).
        margin_c = int(math.ceil(margin_m / self._res))
        rmin = max(0, min(sr, gr) - margin_c)
        rmax = min(self._h, max(sr, gr) + margin_c + 1)
        cmin = max(0, min(sc, gc) - margin_c)
        cmax = min(self._w, max(sc, gc) + margin_c + 1)
        sub = self._grid[rmin:rmax, cmin:cmax]

        # 2) traversable (free) mask. First carve out obstacles not in the occupancy
        #    map (vehicles etc.), then erode (inflate) by the robot radius.
        trav = sub == 0
        if extra_obstacles:
            self._block_obstacles(trav, extra_obstacles, rmin, cmin)
        infl_c = int(round(inflation_m / self._res))
        if infl_c > 0:
            trav = self._erode(trav, infl_c)

        # crop-local coordinates. Record whether the goal cell is traversable (after
        # carve/erosion) -- if it is blocked, do not overwrite the last waypoint with the
        # exact goal (so the robot does not charge back into the vehicle).
        ls, lg = (sr - rmin, sc - cmin), (gr - rmin, gc - cmin)
        H, W = trav.shape
        goal_free = (0 <= lg[0] < H and 0 <= lg[1] < W and bool(trav[lg[0], lg[1]]))
        ls = self._snap(trav, ls, snap_m)
        lg = self._snap(trav, lg, snap_m)
        if ls is None or lg is None:
            return None

        # 3) A*
        cells = self._astar(trav, ls, lg)
        if not cells:
            return None

        # 4) line-of-sight simplification -> convert to world coordinates
        cells = self._simplify(trav, cells)
        path = [self._c2w(r + rmin, c + cmin) for (r, c) in cells]

        # Replace the last waypoint with the exact goal only when the goal cell is traversable.
        # Remove the first waypoint (the current-position cell).
        if path:
            if goal_free:
                path[-1] = (float(goal_xy[0]), float(goal_xy[1]))
            if len(path) > 1:
                path = path[1:]
        return path

    def _block_obstacles(self, trav, obstacles, rmin, cmin):
        """Mark each (x, y, radius) disc from extra_obstacles as not traversable in trav."""
        H, W = trav.shape
        for (wx, wy, rad) in obstacles:
            r, c = self._w2c(wx, wy)
            lr, lc = r - rmin, c - cmin
            rad_c = int(round(float(rad) / self._res))
            if rad_c <= 0:
                continue
            if lr + rad_c < 0 or lr - rad_c >= H or lc + rad_c < 0 or lc - rad_c >= W:
                continue  # outside the crop
            r0, r1 = max(0, lr - rad_c), min(H, lr + rad_c + 1)
            c0, c1 = max(0, lc - rad_c), min(W, lc + rad_c + 1)
            yy, xx = np.ogrid[r0:r1, c0:c1]
            disc = (yy - lr) ** 2 + (xx - lc) ** 2 <= rad_c ** 2
            trav[r0:r1, c0:c1][disc] = False

    # -- internal helpers --

    def _in_bounds(self, r, c):
        return 0 <= r < self._h and 0 <= c < self._w

    @staticmethod
    def _erode(mask, iters):
        """8-neighbor erosion, iters times. Border cells are treated as blocked (protected by the crop margin)."""
        g = mask
        for _ in range(iters):
            e = np.zeros_like(g)
            e[1:-1, 1:-1] = (
                g[1:-1, 1:-1] & g[:-2, 1:-1] & g[2:, 1:-1]
                & g[1:-1, :-2] & g[1:-1, 2:]
                & g[:-2, :-2] & g[:-2, 2:] & g[2:, :-2] & g[2:, 2:]
            )
            g = e
        return g

    @staticmethod
    def _snap(trav, cell, snap_m, res_cells=None):
        """If the cell is not traversable, return the nearest free cell via a spiral search."""
        r, c = cell
        H, W = trav.shape
        if not (0 <= r < H and 0 <= c < W):
            return None
        if trav[r, c]:
            return cell
        # snap_m is in meters, but here it is approximated as a cell-grid radius
        max_rad = int(max(1, round(snap_m / 0.05)))
        for rad in range(1, max_rad + 1):
            for dr in range(-rad, rad + 1):
                for dc in range(-rad, rad + 1):
                    if max(abs(dr), abs(dc)) != rad:
                        continue
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < H and 0 <= nc < W and trav[nr, nc]:
                        return (nr, nc)
        return None

    @staticmethod
    def _astar(grid, start, goal):
        """8-connected A*. A cell list (start..goal) or None."""
        H, W = grid.shape

        def h(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])

        openh = [(h(start, goal), 0.0, start)]
        gscore = {start: 0.0}
        came = {}
        while openh:
            _, gc, cur = heapq.heappop(openh)
            if cur == goal:
                path = [cur]
                while cur in came:
                    cur = came[cur]
                    path.append(cur)
                return path[::-1]
            if gc > gscore.get(cur, 1e18):
                continue
            r, c = cur
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0:
                        continue
                    nr, nc = r + dr, c + dc
                    if not (0 <= nr < H and 0 <= nc < W) or not grid[nr, nc]:
                        continue
                    # Diagonal moves only when both orthogonal cells are free (prevents corner cutting)
                    if dr != 0 and dc != 0 and not (grid[r + dr, c] and grid[r, c + dc]):
                        continue
                    step = SQRT2 if (dr and dc) else 1.0
                    ng = gc + step
                    nb = (nr, nc)
                    if ng < gscore.get(nb, 1e18):
                        gscore[nb] = ng
                        came[nb] = cur
                        heapq.heappush(openh, (ng + h(nb, goal), ng, nb))
        return None

    @classmethod
    def _simplify(cls, grid, path):
        """Merge straight segments via line-of-sight to reduce the number of waypoints."""
        if len(path) <= 2:
            return path
        out = [path[0]]
        i = 0
        n = len(path)
        while i < n - 1:
            j = n - 1
            while j > i + 1 and not cls._los(grid, path[i], path[j]):
                j -= 1
            out.append(path[j])
            i = j
        return out

    @staticmethod
    def _los(grid, a, b):
        """Check whether the straight line a->b is entirely free, using Bresenham."""
        r0, c0 = a
        r1, c1 = b
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r1 > r0 else -1
        sc = 1 if c1 > c0 else -1
        err = dr - dc
        r, c = r0, c0
        while True:
            if not grid[r, c]:
                return False
            if (r, c) == (r1, c1):
                return True
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r += sr
            if e2 < dr:
                err += dr
                c += sc


# ---------------------------------------------------------------------------
# Demo obstacles + Stage 2 answer, read from mock_demo_data.yaml
# (data pre-extracted by tools/mock_data_builder instead of the scenario).
# ---------------------------------------------------------------------------
import os
import logging

import yaml

log = logging.getLogger("participant_app")


def _default_data_path() -> str:
    """demo/mock_demo_data.yaml next to this module."""
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, "mock_demo_data.yaml")


def load_mock_data(path: str | None = None) -> dict:
    """Load the pre-extracted demo data (obstacles + Stage 2 answer)."""
    path = path or os.environ.get("MOCKUP_DEMO_DATA") or _default_data_path()
    if not os.path.exists(path):
        log.warning("[mock-data] not found: %s -- run tools/mock_data_builder/gen_mock_demo_data.py", path)
        return {}
    with open(path) as f:
        return yaml.safe_load(f) or {}


def load_obstacles(path: str | None = None) -> list:
    """Return a list of obstacles [(x, y, radius_m), ...] from mock_demo_data.yaml.

    The runtime occupancy map omits 3D obstructions (vehicles/bicycles/people/objects),
    so the mock navigation injects these to route around them (demo only).
    """
    data = load_mock_data(path)
    obstacles = [(float(o["x"]), float(o["y"]), float(o["radius"]))
                 for o in (data.get("obstacles") or [])]
    log.info("[mock-data] loaded %d obstacles", len(obstacles))
    return obstacles


def obstacles_excluding_goal(obstacles, goal_xy, exclude_m=0.6):
    """Return the list with obstacles within exclude_m of the goal removed.

    Keeps the goal itself (target object / owner zone) reachable, while still routing
    around nearby obstructions.
    """
    gx, gy = float(goal_xy[0]), float(goal_xy[1])
    ex2 = float(exclude_m) ** 2
    out = []
    for (x, y, r) in obstacles:
        if (x - gx) ** 2 + (y - gy) ** 2 <= ex2:
            continue
        out.append((x, y, r))
    return out
