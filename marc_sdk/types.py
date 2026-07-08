"""Data types wrapping MARC 2026 operation-message payloads.

Converts the JSON payloads (dicts) sent by the platform into dataclasses that
are convenient for participants to work with, and serializes the grounding
results a participant submits back into payload dicts.

If you need the raw payload, you can access the original dict via each object's
``.raw`` attribute.
"""

from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class Mission:
    """Stage 1 mission command (msg 201 MISSION_COMMAND)."""

    stage: int
    round: int
    total_rounds: int
    voice_command: str
    time_limit: float
    raw: dict = field(default_factory=dict, repr=False)

    @classmethod
    def from_payload(cls, p: dict) -> "Mission":
        return cls(
            stage=p.get("stage", 1),
            round=p.get("round", 0),
            total_rounds=p.get("total_rounds", 0),
            voice_command=p.get("voice_command", ""),
            time_limit=float(p.get("time_limit", 0.0)),
            raw=p,
        )


@dataclass
class Stage2Mission:
    """Stage 2 mission command (msg 211 STAGE2_MISSION)."""

    task_description: str
    time_limit: float
    stage: int = 2
    # Delivery target (owner zone) coordinate [x, y, z]. The owner location the
    # platform provides together with the VLA command - after picking, the lost
    # item is delivered to this coordinate. None if the scenario does not define
    # an owner_zone.
    owner_position: Optional[List[float]] = None
    raw: dict = field(default_factory=dict, repr=False)

    @classmethod
    def from_payload(cls, p: dict) -> "Stage2Mission":
        op = p.get("owner_position")
        return cls(
            task_description=p.get("task_description", ""),
            time_limit=float(p.get("time_limit", 0.0)),
            stage=p.get("stage", 2),
            owner_position=([float(v) for v in op] if op else None),
            raw=p,
        )


@dataclass
class Score:
    """Scoring result (msg 401 SCORE_RESULT).

    If ``stage == "final"``, this is the final result (stage1_average / stage2 /
    total); otherwise it is a Stage 1 round result (round / scores.total).
    """

    stage: object  # int (round) or "final"
    scores: dict
    round: Optional[int] = None
    seq: Optional[int] = None
    raw: dict = field(default_factory=dict, repr=False)

    @property
    def is_final(self) -> bool:
        return self.stage == "final"

    @property
    def total(self) -> Optional[float]:
        return self.scores.get("total")

    @classmethod
    def from_payload(cls, p: dict, seq: Optional[int] = None) -> "Score":
        return cls(
            stage=p.get("stage"),
            scores=p.get("scores", {}),
            round=p.get("round"),
            seq=seq,
            raw=p,
        )


@dataclass
class Stage2Reveal:
    """Data revealed after Stage 2 grounding scoring (msg 411 STAGE2_REVEAL).

    The reveal the platform sends right after grounding scoring - it tells the
    participant the *approximate* location and type of the correct object so
    that pick/delivery can be performed regardless of grounding accuracy.
    ``hint_center`` is not the exact ground truth but the center of a
    ``hint_radius`` radius; because distractors may be present within the
    radius, use ``target_type`` to decide which object to pick.
    """

    grounding_score: Optional[float]     # score for this grounding submission (0-100)
    target_type: str                     # type of object to pick
    hint_center: List[float]             # approximate location center [x, y, z] (not exact GT)
    hint_radius: float                   # approximate location radius (m)
    raw: dict = field(default_factory=dict, repr=False)

    @classmethod
    def from_payload(cls, p: dict) -> "Stage2Reveal":
        return cls(
            grounding_score=p.get("grounding_score"),
            target_type=p.get("target_type", ""),
            hint_center=list(p.get("hint_center") or []),
            hint_radius=float(p.get("hint_radius", 0.0)),
            raw=p,
        )


@dataclass
class GroundingResult:
    """VLA interpretation result - payload for msg 301 / msg 311 (see the API Reference grounding payload).

    Filled in by the participant and passed to ``client.submit_grounding(...)``.
    Build one from a payload dict via ``from_payload(...)``; ``to_payload()``
    produces the dict for transmission.
    """

    camera_id: str
    target_type: str                     # single target type. For person problems, "person"
    anchor_coord: List[float]            # [x, y, z] - single 3D coordinate
    target_coord: List[float]            # [x, y, z] - single 3D coordinate
    landmark: str = ""                   # reference landmark prefix (when a relation is present)
    relation: Optional[str] = None       # spatial relation term (lost-item relation interpretation)
    situation: Optional[str] = None      # situation category (emergency / abnormal-situation person)

    @staticmethod
    def _coord3(c) -> List[float]:
        """Normalize a coordinate into a 3D float list. 2D input is padded with z=0."""
        vals = [float(v) for v in (c or [])]
        while len(vals) < 3:
            vals.append(0.0)
        return vals[:3]

    def to_payload(self) -> dict:
        interpretation = {
            "target_type": self.target_type,
            "landmark": self.landmark,
        }
        if self.relation is not None:
            interpretation["relation"] = self.relation
        if self.situation is not None:
            interpretation["situation"] = self.situation
        return {
            "camera_id": self.camera_id,
            "interpretation": interpretation,
            "grounding": {
                "anchor_coord": self._coord3(self.anchor_coord),
                "target_coord": self._coord3(self.target_coord),
            },
        }

    @classmethod
    def from_payload(cls, p: dict) -> "GroundingResult":
        interp = p.get("interpretation", {})
        g = p.get("grounding", {})
        return cls(
            camera_id=p.get("camera_id", ""),
            target_type=interp.get("target_type", ""),   # single string
            anchor_coord=list(g.get("anchor_coord", [0.0, 0.0, 0.0])),
            target_coord=list(g.get("target_coord", [0.0, 0.0, 0.0])),
            landmark=interp.get("landmark", ""),
            relation=interp.get("relation"),
            situation=interp.get("situation"),
        )
