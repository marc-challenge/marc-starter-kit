"""Authentication token / handshake proof utilities.

Your ``team_id`` and token are issued to your team out of band; you do not
derive them yourself. The token is your long-term team secret: it is used
**only as the HMAC key** in the challenge-response handshake and is never
transmitted over the network. The only value that travels over the wire is the
expiring ``session_key`` that the platform issues fresh on each connection. The
SDK and the platform compute/verify ``hmac_proof`` with the same rule.

See the developer guide's API Reference, "Handshake (session authentication)",
for the message flow.
"""

import hashlib
import hmac


def hmac_proof(secret: str, server_nonce: str, client_nonce: str,
               team_id: str) -> str:
    """Compute the challenge-response proof.

    ``proof = HMAC-SHA256(secret, "server_nonce|client_nonce|team_id")``.
    ``secret`` is the team token and is never transmitted. This follows the
    **same rule** the platform uses to verify the proof.
    """
    body = f"{server_nonce}|{client_nonce}|{team_id}".encode()
    return hmac.new((secret or "").encode(), body, hashlib.sha256).hexdigest()
