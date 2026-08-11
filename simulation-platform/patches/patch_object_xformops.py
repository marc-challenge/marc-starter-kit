#!/usr/bin/env python3
# Copyright (c) 2026, IoT Convergence & Open Sharing System (IoTCOSS)
#
# MARC 2026 - asset patch: mission-object xformOp order
# ---------------------------------------------------------------------------
# Some mission-object assets ship with NO xformOps on their default prim. The deployer authors
# the transform in two steps - scale first (_apply_asset_scale), then translate
# (_set_prim_translate) - and each step APPENDS its op when none exists. On the very first
# spawn IsaacSimSpawnPrim authors [translate, rotateZYX, scale] itself, so the order is right.
# On a re-spawn at the same prim path (dataset-gen Shuffle / Switch Camera, manip-trainer
# respawn) it authors nothing, so the two steps produce [scale, translate] - and USD then
# applies the translation THROUGH the scale, i.e. the object lands at position * scale:
#   cracker_box (scale 0.8): (-37.9, 122.8, 17.3) -> (-30.3, 98.2, 13.8)  = below ground
#   mug         (scale 1.5): (-45.8, 128.8, 17.3) -> (-68.8, 193.2, 26.0) = outside the FOV
# The object is still spawned and labelled, it is just nowhere near the camera, so every scene
# after the first one loses it from the ground truth.
#
# Assets that already carry [translate, orient, scale] on their default prim (cola_can, juice,
# ...) are unaffected: both steps find their op and only Set() it, leaving the order intact.
# Landmarks and people are unaffected too - their deploy path always rebuilds the op order.
#
# This patch gives the op-less assets the same identity [translate, orient, scale] the working
# assets have, so the deployer only ever Set()s existing ops. boot.py runs it in a separate
# process before the tool starts. It is idempotent: assets that already have xformOps are
# skipped, so it becomes a no-op once the content image ships assets with the ops authored.
import glob
import os
import sys

CONFIG_PATH = "/metacom2026/scenarios/config.yaml"
ASSETS_DIR = "/metacom2026/resources/assets"
SECTION = "mission_object_assets"


def _enable_pxr():
    """Make IsaacSim's bundled USD libraries importable.

    python.sh does not set up the kit extension paths, so `import pxr` fails without this.
    """
    for d in sorted(glob.glob("/isaac-sim/extscache/omni.usd.libs-*")):
        if d not in sys.path:
            sys.path.insert(0, d)


def main():
    if not os.path.exists(CONFIG_PATH):
        print(f"[xformops-patch] config not found, skipping: {CONFIG_PATH}")
        return 0

    _enable_pxr()
    try:
        import yaml
        from pxr import Gf, Usd, UsdGeom
    except ImportError as e:
        print(f"[xformops-patch] import failed, skipping: {e}")
        return 0

    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        assets = (yaml.safe_load(f) or {}).get(SECTION) or {}

    patched = []
    for name, entry in assets.items():
        usd_file = (entry or {}).get("usd_file")
        if not usd_file:
            continue
        path = os.path.join(ASSETS_DIR, usd_file)
        if not os.path.exists(path):
            print(f"[xformops-patch] asset not found, skipping: {name} ({usd_file})")
            continue

        # One unwritable/unreadable asset must not stop the remaining ones.
        try:
            stage = Usd.Stage.Open(path)
            if stage is None:
                print(f"[xformops-patch] could not open USD, skipping: {name}")
                continue
            prim = stage.GetDefaultPrim()
            if not prim or not prim.IsValid() or not prim.IsA(UsdGeom.Xformable):
                print(f"[xformops-patch] no xformable default prim, skipping: {name}")
                continue

            xformable = UsdGeom.Xformable(prim)
            if xformable.GetOrderedXformOps():
                continue  # already has ops - the deployer will Set() them, order stays correct

            translate_op = xformable.AddTranslateOp()
            translate_op.Set(Gf.Vec3d(0.0, 0.0, 0.0))
            orient_op = xformable.AddOrientOp()
            orient_op.Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
            scale_op = xformable.AddScaleOp()
            scale_op.Set(Gf.Vec3f(1.0, 1.0, 1.0))
            # The Add* calls already produce this order; make the invariant explicit.
            xformable.SetXformOpOrder([translate_op, orient_op, scale_op])
            stage.GetRootLayer().Save()
        except Exception as e:
            print(f"[xformops-patch] failed, skipping: {name} ({e})")
            continue
        patched.append(name)

    if patched:
        print(f"[xformops-patch] authored identity xformOps on {len(patched)} asset(s): "
              f"{', '.join(patched)}")
    else:
        print("[xformops-patch] nothing to patch (all assets already have xformOps)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
