"""Mock VLA agent -- stands in for the participant's perception/grounding model.

A real participant would analyse the CCTV images and reason about the voice command to
produce a grounding answer. This mock does none of that: it looks up a pre-extracted
answer for the received voice_command in ``mock_demo_data.yaml`` (built by
``tools/mock_data_builder/gen_mock_demo_data.py``). Those answers are the scenario
ground truth with a few random wrong axes mixed in, so the demo submits plausible but
imperfect answers and earns partial scores instead of always scoring 1.0.

``process()`` / ``process_stage2()`` return a ``GroundingResult`` (built from the stored
payload via ``GroundingResult.from_payload``), so the result can be passed straight to
``client.submit_grounding(result)`` / ``client.submit_stage2_grounding(result)``.

Replace this class with a real VLA model that returns a ``GroundingResult`` from the CCTV
images to build an actual competition agent.
"""

import logging
import os

import yaml

from marc_sdk import GroundingResult

log = logging.getLogger("participant_app")


def _default_data_path() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, "mock_demo_data.yaml")


class MockVLAGrounding:
    """Look up a pre-extracted grounding answer by voice_command.

    The answers live in ``mock_demo_data.yaml`` under ``stage1_answers`` (Stage 1
    grounding problems) and ``stage2_answers`` (Stage 2 retrieval problems), keyed by the
    exact voice_command / task text of each problem.
    """

    def __init__(self, data_path: str = None):
        path = data_path or os.environ.get("MOCKUP_DEMO_DATA") or _default_data_path()
        data = {}
        if os.path.exists(path):
            with open(path) as f:
                data = yaml.safe_load(f) or {}
        else:
            log.warning("[mock-vla] %s not found -- run tools/mock_data_builder/"
                        "gen_mock_demo_data.py to generate it", path)
        self._stage1 = {self._key(k): v for k, v in (data.get("stage1_answers") or {}).items()}
        self._stage2 = {self._key(k): v for k, v in (data.get("stage2_answers") or {}).items()}

    @staticmethod
    def _key(s: str) -> str:
        return (s or "").strip().lower()

    def _lookup(self, table: dict, command: str, kind: str) -> GroundingResult:
        payload = table.get(self._key(command))
        if payload is None:
            log.warning("[mock-vla] no %s answer for %r -- submitting empty grounding",
                        kind, command)
            payload = {"camera_id": "", "interpretation": {}, "grounding": {}}
        return GroundingResult.from_payload(payload)

    def process(self, voice_command: str, camera_ids: list = None) -> GroundingResult:
        """Stage 1: return the pre-extracted answer for this voice_command.

        ``camera_ids`` is accepted for signature parity with a real VLA (which would need
        the CCTV list) but is unused here -- the answer already carries its camera_id.
        """
        return self._lookup(self._stage1, voice_command, "stage1")

    def process_stage2(self, task_description: str) -> GroundingResult:
        """Stage 2: return the pre-extracted answer for this task command."""
        return self._lookup(self._stage2, task_description, "stage2")
