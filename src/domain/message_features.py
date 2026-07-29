from enum import Enum
from src.core.message import Message

class StrategicDecision(Enum):
    APPROVE = "APPROVE"
    COUNTER_PROPOSAL = "COUNTER_PROPOSAL"
    REJECT = "REJECTED"

class MessageType(Enum):
    PERIODIC_REPORT = 1
    PROPOSAL = 2
    SD_RESPONSE = 3
    CONFLICT_PREDICTIONS = 4
    REQUEST_CONFLICT_PREDICTION = 5