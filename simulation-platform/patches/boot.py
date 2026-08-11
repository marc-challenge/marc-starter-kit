#!/usr/bin/env python3
# Copyright (c) 2026, IoT Convergence & Open Sharing System (IoTCOSS)
#
# MARC 2026 - boot wrapper: apply asset patches, then run the tool unchanged.
# ---------------------------------------------------------------------------
# The image ENTRYPOINT is /isaac-sim/python.sh, so setting the compose command to
#   ["/marc-patches/boot.py", "<original entry script>", ...]
# makes python.sh run this file, which patches and then hands off to the original entry.
# platform / dataset-gen / manip-trainer share one image, so they share this wrapper.
#
# Patches run in a SEPARATE process so the pxr (USD) libraries are never loaded into this
# process - that keeps them out of IsaacSim's own initialization. The handoff uses os.execv,
# so the process tree and signal handling are identical to running without the wrapper.
import glob
import os
import subprocess
import sys

PYTHON_SH = "/isaac-sim/python.sh"
DEFAULT_TARGET = "/metacom2026/simulation_app/marc2026/runtime.py"
PATCH_DIR = os.path.dirname(os.path.abspath(__file__))
PATCHES = ["patch_lidar_frameid.py", "patch_object_xformops.py", "patch_object_rotation.py"]


def _patch_env():
    """Add IsaacSim's bundled USD native libraries to LD_LIBRARY_PATH for the patch process."""
    env = os.environ.copy()
    libs = []
    for d in sorted(glob.glob("/isaac-sim/extscache/omni.usd.libs-*")):
        libs += [f"{d}/bin", f"{d}/lib"]
    libs.append("/isaac-sim/kit/libs")
    env["LD_LIBRARY_PATH"] = ":".join(libs + [env.get("LD_LIBRARY_PATH", "")])
    return env


def main():
    argv = sys.argv[1:]
    target = argv[0] if argv else DEFAULT_TARGET
    rest = argv[1:]

    env = _patch_env()
    for name in PATCHES:
        patch = os.path.join(PATCH_DIR, name)
        if not os.path.exists(patch):
            print(f"[boot] patch not found, skipping: {patch}", flush=True)
            continue
        # A failing patch must not block startup - the asset may already be correct.
        r = subprocess.run([PYTHON_SH, patch], env=env)
        if r.returncode != 0:
            print(f"[boot] patch failed (continuing): {name} rc={r.returncode}", flush=True)

    print(f"[boot] starting: {target}", flush=True)
    os.execv(PYTHON_SH, [PYTHON_SH, target] + rest)


if __name__ == "__main__":
    main()
