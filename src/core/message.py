from uuid import uuid4

class Message:
    def __init__(self, source, destination,message_type,payload,issue_timestamp, correlation_id = 0):
        self.source = source
        self.destination = destination
        self.message_type = message_type
        self.payload = payload
        self.message_id = str(uuid4())
        self.issue_timestamp = issue_timestamp
        self.correlation_id = correlation_id

    def __str__(self):
        return (
            f"Source: {self.source}\n"
            f"Destination: {self.destination}\n"
            f"Msg Type: {self.message_type}\n"
            f"Payload: {self.payload}\n"
            f"Issue Timestamp: {self.issue_timestamp}\n"
            f"Message ID: {self.message_id}\n"
            f"Correlation ID: {self.correlation_id}"
        )