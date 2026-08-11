#!/usr/bin/env python3
# Copyright (c) 2026, IoT Convergence & Open Sharing System (IoTCOSS)
#
# MARC 2026 - source patch: mission-object rotation + xformOp order
# ---------------------------------------------------------------------------
# Companion to patch_object_xformops.py, which fixes the same root cause from the asset side.
#
# deploy_mission_objects() hands the object rotation to IsaacSimSpawnPrim and never authors it
# itself. That works on the FIRST spawn of a prim path, but on a re-spawn at the same path
# (dataset-gen Shuffle / Switch Camera, manip-trainer respawn) the command authors no transform
# at all, so:
#   - the random 3-axis rotation is silently dropped (objects spawn axis-aligned), and
#   - _apply_asset_scale / _set_prim_translate then APPEND their ops, producing the order
#     [scale, translate] instead of [translate, ..., scale] - which makes USD apply the
#     translation through the scale, i.e. the object lands at position * scale.
# deploy_landmarks() does not have this problem: it always follows up with
# _set_landmark_rotation / _set_fallen_rotation, and both rebuild the op order explicitly.
#
# This patch gives objects the same treatment: author the rotation as an orient op (the SAME
# quaternion already handed to IsaacSimSpawnPrim, so no pose changes anywhere) and rebuild the
# order to [translate, orient, scale].
#
# boot.py runs this in a separate process before the tool starts. It is idempotent (the marker
# below) and fails safe: if the anchor does not match - e.g. a newer content image already
# fixed this - it prints and leaves the file untouched.
import os
import re
import sys

TARGET = "/metacom2026/simulation_app/marc2026/world/object_deployer.py"
MARKER = "_set_object_orientation"

# deploy_mission_objects() only - the 12-space indent distinguishes it from the deploy_landmarks
# call site (8 spaces). Comment lines in between are tolerated.
ANCHOR = re.compile(
    r'(?m)^            _apply_asset_scale\(prim_path, asset\["scale"\]\)\n'
    r'(?:^            #.*\n)*'
    r'^            _set_prim_translate\(prim_path, position\)\n'
)

CALL = '''            # [marc-patch] On a re-spawn (same prim path recreated) IsaacSimSpawnPrim
            # authors no transform, so the random rotation is lost and scale/translate are
            # appended, flipping the op order to [scale, translate] - which makes the
            # position come out multiplied by the scale. Author the rotation explicitly and
            # rebuild the order as [translate, orient, scale].
            _set_object_orientation(prim_path, quats)
'''

HELPER = '''

def _set_object_orientation(prim_path, quats):
    """[marc-patch] Author the object rotation as an orient op and rebuild the op order.

    quats is the euler_angles_to_quats() result [x, y, z, w] - the same rotation handed to
    IsaacSimSpawnPrim, so the pose of a first-time placement does not change. Any rotate* op
    left behind by the asset or the spawn command is dropped from the order to avoid applying
    the rotation twice (the same approach as _set_fallen_rotation).
    """
    from pxr import UsdGeom, Gf
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return

    xformable = UsdGeom.Xformable(prim)
    existing = {op.GetOpName(): op for op in xformable.GetOrderedXformOps()}
    orient_op = existing.get("xformOp:orient") or xformable.AddOrientOp()
    orient_op.Set(Gf.Quatf(float(quats[3]),
                           Gf.Vec3f(float(quats[0]), float(quats[1]), float(quats[2]))))
    ordered = []
    if "xformOp:translate" in existing:
        ordered.append(existing["xformOp:translate"])
    ordered.append(orient_op)
    if "xformOp:scale" in existing:
        ordered.append(existing["xformOp:scale"])
    xformable.SetXformOpOrder(ordered)
'''


def main():
    if not os.path.exists(TARGET):
        print(f"[rotation-patch] source not found, skipping: {TARGET}")
        return 0

    with open(TARGET, "r", encoding="utf-8") as f:
        src = f.read()

    if MARKER in src:
        print("[rotation-patch] already applied, skipping")
        return 0

    matches = ANCHOR.findall(src)
    if len(matches) != 1:
        print(f"[rotation-patch] anchor matched {len(matches)} time(s) (expected 1), "
              "skipping - source may already be fixed upstream")
        return 0

    src = ANCHOR.sub(lambda m: m.group(0) + CALL, src, count=1) + HELPER
    try:
        with open(TARGET, "w", encoding="utf-8") as f:
            f.write(src)
    except OSError as e:
        print(f"[rotation-patch] write failed, skipping: {e}")
        return 0

    print("[rotation-patch] deploy_mission_objects: object rotation authored explicitly "
          "(xformOp order -> [translate, orient, scale])")
    return 0


if __name__ == "__main__":
    sys.exit(main())
