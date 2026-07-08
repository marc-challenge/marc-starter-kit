# NOTICES - MARC 2026 Starter Kit (third-party assets and execution policy notice)

## Execution environment / internet policy
- **The competition (judging) runtime has no internet access.** Public APIs and runtime external downloads are prohibited.
- **However, internet access is available at build time** - model weights and dependencies are **baked in at build time** (no runtime dependency).
- Preliminary scoring hardware baseline: Ubuntu 22.04 / i7-12700K / 125GiB / RTX PRO 5000 (48GB).

## IsaacSim / NVIDIA (distribution form)
- The platform/Trainer images are distributed as **Dockerfile-only** (prebuilt images are not distributed).
  The IsaacSim container is subject to the *NVIDIA Isaac Sim Additional Software and Materials License*,
  which prohibits external redistribution of derivative binary images (section 2.2, section 2.4).
- Participants **pull `nvcr.io/nvidia/isaac-sim:5.1.0` under their own name** (accepting the EULA = they are the licensee) and then
  **build locally themselves**. We do not redistribute the NVIDIA binaries.
- NVIDIA `IsaacSim-dockerfiles` is Apache-2.0 (writing/publishing the Dockerfile is allowed; the restriction applies only to the built image).

## USD / playground assets
- The published background (playground) USD = **chungmu only (363M)**. Mission objects/landmark USD included.
- **"The competition background location (playground) may change"** - the public build only has the practice chungmu, and the actual competition
  may run on a different background (to prevent overfitting / stage exposure).

## Software

| Item | Where used | License |
|---|---|---|
| NVIDIA Isaac Sim | Simulation platform (built locally, not redistributed) | NVIDIA Isaac Sim Additional Software and Materials License (see above) |
| ROS 2 (Humble) + `rclpy`, std/sensor/geometry/nav messages | Platform ↔ agent communication | Apache-2.0 |
| Participant SDK (`marc_sdk`) | This kit | MIT |

## 3D asset attributions

Only the converted USD assets used at runtime are shipped (original source files such as `.glb` are not
retained). The assets are delivered inside the platform container image, so the attributions below are
self-contained here.

- **People characters** — **CC0 1.0** (Public Domain).
- **Robot model** (`manipulation/urdf/gen3_6dof_vision_2f140.urdf`) — derived from Kinova Gen3 + Robotiq 2F-140 ROS descriptions (typically BSD-3); attribution finalized by the organizer at release.

### Vehicles — CC0 1.0 (Public Domain)
LowPoly cars by **@Quaternius** under **CC0 1.0** — `landmark/car/{police_car, normal_car_1, normal_car_2, sports_car, sports_car_2, suv, taxi}`. The notice bundled with the assets reads:

> LowPoly Models by @Quaternius
> Consider supporting me on Patreon, even $1 helps me a lot!
> https://www.patreon.com/quaternius
>
> License: CC0 1.0 Universal (CC0 1.0) Public Domain Dedication
> https://creativecommons.org/publicdomain/zero/1.0/

### Objects — CC BY (Creative Commons Attribution)
From Sketchfab / Fab under **CC BY**; attribution to the original author is required (see the source link).
Some assets are modified derivatives (e.g. resized or re-textured); the original author is still credited.

| Category | Asset | Source |
|---|---|---|
| object/etc | closedlongumbrella | https://sketchfab.com/3d-models/umbrella-not-open-b5835433a6414388a9846568cc110da |
| object/etc | pencilcase (modified) | https://sketchfab.com/3d-models/pencil-case-pencil-sharpener-eraser-and-ruler-62d50825c5de4d1ba574356fbe9b2985 |
| object/etc | sunblock (modified) | https://sketchfab.com/3d-models/kolagra-sunblock-tube-d3b9b069af3a4d78915ebfddccea5dbb |
