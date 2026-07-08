# MARC 2026 Starter Kit

Starter kit for the **MARC 2026 (MetaSejong AI Robot Challenge)**. It contains everything you need to
build your agent: the simulation platform (Docker recipes), the participant SDK (`marc_sdk`), a runnable
demo, and the manipulation reference code.

Because the Isaac Sim binary cannot be redistributed, the platform ships as **Docker recipes** that you
build on your own machine (after logging in to NVIDIA NGC). One unified image serves all three tools --
simulation platform, dataset generator, and manipulation trainer -- selected at run time.

## Quick start

```bash
docker login nvcr.io                         # your own free NGC account (accept the EULA)
bash simulation-platform/marc.sh setup       # build the base image (once)
bash simulation-platform/marc.sh platform    # run the simulation platform
```

Then, on a separate machine, run your agent (see `demo/`).

## Documentation

The developer guide is the single source of truth for setup, the SDK/API reference, and submission:

- **Developer guide:** https://marc-challenge.github.io/marc-dev-guide/
- **Competition homepage:** https://marc-challenge.github.io/

## What's inside

| Path | What it is |
|---|---|
| `simulation-platform/` | Platform Docker recipes + `marc.sh` wrapper (platform / dataset-gen / manip-trainer) |
| `marc_sdk/` | Participant SDK -- talk to the platform (sensors, robot control, answer submission) |
| `demo/` | Runnable reference agent. **The mock answers are examples -- replace them with your real inference before submission.** |
| `manipulation/` | Robot-arm kinematics reference (URDF + FK/IK + baseline pick) |

See the developer guide for the full walkthrough.
