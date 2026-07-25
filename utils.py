class Message:
    def __init__(self, source, destination,message_type,payload,issue_timestamp):
        self.source = source
        self.destination = destination
        self.message_type = message_type
        self.payload = payload
        self.issue_timestamp = issue_timestamp

    def __str__(self):
        return f"""Source: {self.source}
                Destination: {self.destination}
                Msg Type: {self.message_type}
                Payload: {self.payload}
                """
        


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
        self.x = x
        self.y = y
        self.z = z
        self.t = t

    def __str__(self):
        return f"[{self.x}, {self.y}, {self.z}, {self.t}]"

    def __gt__(self, other):
        if self.t > other.t:
            return True
        else:
            return False