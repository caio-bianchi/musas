from uuid import uuid4
import math


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