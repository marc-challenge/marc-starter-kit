#!/usr/bin/env bash
# Copyright (c) 2026, IoT Convergence & Open Sharing System (IoTCOSS)
#
# MARC 2026 - participant run wrapper (convenience + preflight). The canonical path is docker compose.
# ---------------------------------------------------------------------------
# Call: `bash simulation-platform/marc.sh <cmd>` (no chmod/CRLF dependency). Relative to starter-kit repo root.
#   setup     nvcr.io login guidance + xhost (GUI) + base image build (once)
#   platform  (build if needed then) platform profile up   - runtime platform (GHCR content image)
#   dataset-gen  dataset-gen profile up                - generate object-detection training set (labels) (GHCR content image)
#   manip-trainer  manipulation practice environment (manipulation_trainer.py, reuses the dataset-gen image)
#   select [SCENARIO]  pick which problems the platform poses -> writes problems.yaml (in-container TUI)
#   demo <id> run the SDK demo participant (separate machine/shell, -> ../demo)
#   down      stop all profiles
#   clean     stop + remove locally-built images
#
# If the wrapper does not run in your environment, run compose directly:
#   docker compose -f simulation-platform/docker-compose.yml --profile platform up --build
# ---------------------------------------------------------------------------
set -euo pipefail

# Move to the starter-kit repo root (this script lives under simulation-platform/)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT"

COMPOSE_FILE="simulation-platform/docker-compose.yml"
BASE_IMAGE="marc-base:ros2-isaacsim-5.1"

# Auto-detect docker compose (v2) vs docker-compose (v1)
if docker compose version >/dev/null 2>&1; then
  DC=(docker compose)
elif command -v docker-compose >/dev/null 2>&1; then
  DC=(docker-compose)
else
  echo "ERROR: docker compose (v2) or docker-compose (v1) is required." >&2
  exit 1
fi

preflight() {
  command -v docker >/dev/null 2>&1 || { echo "ERROR: docker not installed"; exit 1; }
  docker info 2>/dev/null | grep -qi nvidia \
    || echo "WARN: nvidia container runtime not detected - GPU may not work (check nvidia-container-toolkit)."
}

ensure_base() {
  if ! docker image inspect "$BASE_IMAGE" >/dev/null 2>&1; then
    echo "[marc] building base image ($BASE_IMAGE) - first time only, this takes a while."
    docker build -f simulation-platform/Dockerfile.base -t "$BASE_IMAGE" .
  fi
}

case "${1:-}" in
  setup)
    preflight
    echo "[marc] nvcr.io login required (free NGC account): docker login nvcr.io"
    command -v xhost >/dev/null 2>&1 && xhost +local:root || true   # for GUI (HEADLESS=false)
    ensure_base
    echo "[marc] setup complete. Next: bash simulation-platform/marc.sh platform"
    ;;
  platform)
    preflight; ensure_base
    echo "[marc] platform: GHCR content image COPY --from."
    "${DC[@]}" -f "$COMPOSE_FILE" --profile platform up --build
    ;;
  dataset-gen)
    preflight; ensure_base
    echo "[marc] dataset-gen: GHCR content image COPY --from."
    "${DC[@]}" -f "$COMPOSE_FILE" --profile dataset-gen up --build
    ;;
  manip-trainer)
    preflight; ensure_base
    echo "[marc] manipulation practice environment (manipulation_trainer.py)."
    "${DC[@]}" -f "$COMPOSE_FILE" --profile manip-trainer up --build
    ;;
  select)
    # Problem-selection TUI, run INSIDE the platform image (reads the scenario from the content image,
    # so no scenario file is shipped in this repo). Writes ./problems.yaml (auto-used on the next run).
    shift
    scenario="${1:-${ENV_MARC_SCENARIO:-marc2026_demo}}"
    # The TUI runs inside the platform image (to read the in-image scenario). Build it if missing
    # - the first select therefore takes a while (one-time image build), then it is instant.
    if ! docker image inspect marc-platform:practice >/dev/null 2>&1; then
      echo "[marc] platform image not found - building it first (one-time; needed to read the scenario)."
      preflight; ensure_base
      "${DC[@]}" -f "$COMPOSE_FILE" build platform
    fi
    touch problems.yaml   # the bind-mount target must exist as a file (else Docker creates a directory)
    echo "[marc] pick problems for '$scenario'; saves ./problems.yaml (auto-detected at run time, no env var)."
    "${DC[@]}" -f "$COMPOSE_FILE" run --rm select "$scenario" -o /metacom2026/problems.yaml
    ;;
  demo)
    shift
    team="${1:-u1}"
    echo "[marc] the demo participant runs on a separate machine/shell, isolated from the platform (-> demo/README.md)."
    docker compose -f demo/docker-compose.yml up --build
    ;;
  down)
    "${DC[@]}" -f "$COMPOSE_FILE" --profile platform --profile dataset-gen --profile manip-trainer down
    ;;
  clean)
    "${DC[@]}" -f "$COMPOSE_FILE" --profile platform --profile dataset-gen --profile manip-trainer down || true
    docker image rm marc-platform:practice 2>/dev/null || true
    echo "[marc] clean complete (base image is kept)."
    ;;
  *)
    echo "usage: bash simulation-platform/marc.sh {setup|platform|dataset-gen|manip-trainer|select [scenario]|demo <id>|down|clean}"
    exit 1
    ;;
esac
