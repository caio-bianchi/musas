class Message:
    def __init__(self, source, destination,message_type,payload,issue_timestamp):
        self.source = source
        self.destination = destination
        self.message_type = message_type
        self.payload = payload
        self.issue_timestamp = issue_timestamp