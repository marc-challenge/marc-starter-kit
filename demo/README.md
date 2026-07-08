# SDK Demo Participant (reference implementation)

A runnable baseline built on **`marc_sdk`**. The protocol and ROS wiring (registration,
subscribe/publish, msg 301/311/302, `cmd_vel`, `world_pose`, occupancy) are all handled by
`MARCClient`, so you only fill in the **perception (VLA grounding) + navigation + manipulation**
logic through callbacks.

> This demo does not actually "solve" anything: it looks up pre-extracted answers and drives to
> pre-computed positions to exercise the full flow. **Replace the mock parts with your real
> perception/decision code before submission.**

## File layout

| File | Description |
|---|---|
| `participant_app.py` | Entry point -- registers `MARCClient` callbacks and wires the full Stage 1 / Stage 2 flow |
| `mock_agent_vla.py` | Mock perception. Instead of analysing CCTV, it looks up the answer for the received voice_command in `mock_demo_data.yaml` and returns a `GroundingResult` |
| `mock_agent_navigation.py` | Stage 2 navigation: goal-seek control + occupancy-grid A* planning + obstacle avoidance |
| `mock_agent_manipulation.py` | Robot-arm pick motion (joint-angle keyframes) |
| `mock_demo_data.yaml` | Demo-only data extracted from the scenario (per-problem answer coordinates + obstacles). Real participants do not have this |
| `Dockerfile`, `docker-compose.yml` | Containerized run (the standard submission form) |
| `launch.sh` | Host run script (sets up ROS 2 Humble + PYTHONPATH) |

The three `mock_agent_*` files stand in for the agent code you will develop
(perception / navigation / manipulation). `participant_app.py` just calls them and connects the flow.

## Running

### A. Docker (standard -- submission method)

The participant application must be **developed/submitted as a Docker image** (required). The standard
entry point is **`docker compose up`**, and the team ID/token are entered in `docker-compose.yml`.

```bash
# Precondition: the simulation platform must already be running (a separate machine for scoring).

# 1) Fill MARC_TEAM_ID / MARC_TOKEN in docker-compose.yml with your assigned values
# 2) Standard run:
cd demo
docker compose up        # builds, then starts the agent (the organizer scores with the same command)
```

- **Separate machines**: the participant app runs on **separate, dedicated hardware** from the platform. The two machines share the same LAN and the **same `ROS_DOMAIN_ID`**, and DDS goes over the network (UDP). `network_mode: host` makes discovery happen on the machine NIC.
- GPU: the participant machine's GPU is used for VLA inference (`deploy.resources` -- requires nvidia-container-toolkit). Remove that block if not needed.
- Because CCTV video is transmitted over the network, a gigabit+ LAN is recommended.
- Submission: push to your private repository's `master` branch, tag the submission commit, and add the `marc-challenge-office` account as a collaborator -> the organizers clone it and run `docker compose up` (teams are scored one at a time, in sequence). Keep the token in the compose file of your private repository only (never in a public repository). See the Submission Guide for the full procedure.

### B. Host run (for development)

```bash
export MARC_TEAM_ID=u1
export MARC_TOKEN=<token>
cd demo
./launch.sh u1
# or set the env vars directly:
#   MARC_TEAM_ID=u1 MARC_TOKEN=<token> python3 participant_app.py
```

Even without `pip install`ing `marc_sdk`, `launch.sh` adds the SDK path (the starter-kit root) to
`PYTHONPATH`, so it runs directly from the kit.

## Behavior (non-interactive / automatic)

1. **Registration** -> SESSION_ACK
2. **Stage 1**: per-round `voice_command` -> mock VLA looks up the answer -> `submit_grounding()` (repeated)
3. **Stage 2**: `task_description` -> mock VLA -> `submit_stage2_grounding()` -> on reveal, occupancy A* to the
   target -> arm pick -> A* to the owner zone -> `task_complete()`
4. **End**: log the final score

## Environment variables

| Variable | Default | Description |
|---|---|---|
| `MARC_TEAM_ID` | `u1` | Team ID |
| `MARC_TOKEN` | -- | Session token (required) |
| `ROS_DOMAIN_ID` | `0` | ROS domain |
