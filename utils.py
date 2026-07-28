from uuid import uuid4
import math

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


# Is it better to state Flight Plan as a class or as a file?
class FlightPlan:
    def __init__(self, waypoint_list):
        waypoint_list.sort()
        self.waypoint_list = waypoint_list

    def add_waypoint(self, wp):
        self.waypoint_list.append(wp)
        self.waypoint_list.sort()

    def print_waypoint_list(self):
        for wp in self.waypoint_list:
            print(wp)

class Waypoint:
    def __init__(self, x, y, z, t):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.t = float(t)

    def position(self):
        return [self.x, self.y, self.z]

    def __str__(self):
        return f"[{self.x}, {self.y}, {self.z}, {self.t}]"

    def __gt__(self, other):
        if self.t > other.t:
            return True
        else:
            return False


def norm(vector):
    return math.sqrt(sum(component**2 for component in vector))

def clip(value, minimum, maximum):
    return max(minimum, min(value, maximum))

def substract(v1, v2):
    return[
        v1_component - v2_component
        for v1_component, v2_component in zip(v1,v2)
    ]
