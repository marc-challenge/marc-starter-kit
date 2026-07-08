"""MARC 2026 participant SDK.

A ROS2 protocol abstraction library for communicating with the MARC
(Meta-Sejong AI Robotics Challenge) 2026 platform. It hides handshaking,
msg_id parsing, seq/session pairing, QoS, and dynamic CCTV discovery behind
``MARCClient`` so that participants can focus on their VLA/navigation algorithms.

See the developer guide's API Reference for the message protocol.

    from marc_sdk import MARCClient
    client = MARCClient.from_env()
    client.connect()
    ...
    client.run()
"""

from .client import MARCClient
from .types import Mission, Stage2Mission, Score, Stage2Reveal, GroundingResult
from . import protocol
from . import auth

__version__ = "0.1.0"

__all__ = [
    "MARCClient",
    "Mission",
    "Stage2Mission",
    "Score",
    "Stage2Reveal",
    "GroundingResult",
    "protocol",
    "auth",
    "__version__",
]
