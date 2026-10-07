from dataclasses import dataclass, field
from typing import Any
from uuid import uuid4

from utm_simulator.domain.message_types import MessageType


@dataclass
class Message:
    source: str
    destination: str
    message_type: MessageType
    payload: dict[str, Any]
    issue_timestamp: float
    in_reply_to: str | None = None
    message_id: str = field(
        default_factory=lambda: str(uuid4()),
        init=False, # Parameter can't be provided to the constructor
    )