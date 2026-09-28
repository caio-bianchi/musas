from typing import Any
from uuid import uuid4

from src.domain.message_features import MessageType


class Message:

    def __init__(
        self,
        source: str,
        destination: str,
        message_type: MessageType,
        payload: dict[str, Any],
        issue_timestamp: float,
        in_reply_to: str | None = None,
    ) -> None:
        self.source: str = source
        self.destination: str = destination
        self.message_type: MessageType = message_type
        self.payload: dict[str, Any] = payload
        self.message_id: str = str(uuid4())
        self.issue_timestamp: float = issue_timestamp
        self.in_reply_to: str | None = in_reply_to

    def __str__(self) -> str:
        return (
            f"Source: {self.source}\n"
            f"Destination: {self.destination}\n"
            f"Msg Type: {self.message_type}\n"
            f"Payload: {self.payload}\n"
            f"Issue Timestamp: {self.issue_timestamp}\n"
            f"Message ID: {self.message_id}\n"
            f"In Reply To: {self.in_reply_to}"
        )