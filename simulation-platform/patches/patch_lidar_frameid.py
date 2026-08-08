#!/usr/bin/env python3
# Copyright (c) 2026, IoT Convergence & Open Sharing System (IoTCOSS)
#
# MARC 2026 - asset patch: RTX LiDAR publisher frameId
# ---------------------------------------------------------------------------
# The shipped metacombot2.usd still carries 'Velodyne_VLS_128' in
# ROS_LidarRTX/PointCloudPublish.inputs:frameId. That frame is never published on /tf, so
# TF-based consumers (RViz2, tf2 lookups, Nav2) cannot place the point cloud. The topic
# itself IS published normally - only the header label is wrong.
# The real LiDAR frame is 'Base_LiDAR', published as arm_base_link -> Base_LiDAR.
#
# boot.py runs this in a separate process before the tool starts. It is idempotent: once the
# content image ships the corrected value this becomes a no-op.
import glob
import os
import sys

USD_PATH = "/metacom2026/resources/assets/robot/metacombot2_USD/metacombot2.usd"
NODE_PATH = "/metacombot2/graph/ROS_LidarRTX/PointCloudPublish"
ATTR = "inputs:frameId"
WANT = "Base_LiDAR"


def _enable_pxr():
    """Make IsaacSim's bundled USD libraries importable.

    python.sh does not set up the kit extension paths, so `import pxr` fails without this.
    """
    for d in sorted(glob.glob("/isaac-sim/extscache/omni.usd.libs-*")):
        if d not in sys.path:
            sys.path.insert(0, d)


def main():
    if not os.path.exists(USD_PATH):
        print(f"[lidar-patch] USD not found, skipping: {USD_PATH}")
        return 0

    _enable_pxr()
    try:
        from pxr import Usd
    except ImportError as e:
        print(f"[lidar-patch] pxr import failed, skipping: {e}")
        return 0

    stage = Usd.Stage.Open(USD_PATH)
    if stage is None:
        print(f"[lidar-patch] could not open USD, skipping: {USD_PATH}")
        return 0

    prim = stage.GetPrimAtPath(NODE_PATH)
    if not prim.IsValid():
        print(f"[lidar-patch] node not found, skipping: {NODE_PATH}")
        return 0

    attr = prim.GetAttribute(ATTR)
    if not attr:
        print(f"[lidar-patch] attribute not found, skipping: {NODE_PATH}.{ATTR}")
        return 0

    current = attr.Get()
    if current == WANT:
        print(f"[lidar-patch] already '{WANT}' - nothing to do")
        return 0

    attr.Set(WANT)
    stage.GetRootLayer().Save()
    print(f"[lidar-patch] frameId '{current}' -> '{WANT}' (saved)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
