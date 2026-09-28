from src.domain.flight import Waypoint

class Conflict():
    """
    Base class for conflict representations.

    Users may define their own conflict types by subclassing this class.
    """

# Is it better to state Flight Plan as a class or as a file?
class FlightPlan:
    def __init__(self, waypoint_list: list[Waypoint])->None:
        waypoint_list.sort()
        self.waypoint_list = waypoint_list

    def add_waypoint(self, waypoint: Waypoint)->None:
        self.waypoint_list.append(waypoint)
        self.waypoint_list.sort()

    def print_waypoint_list(self)->None:
        for wp in self.waypoint_list:
            print(wp)

class Waypoint:
    def __init__(self, x, y, z, t)->None:
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.t = float(t)

    def position(self)->list[float]:
        return [self.x, self.y, self.z]

    def __str__(self)->None:
        return f"[{self.x}, {self.y}, {self.z}, {self.t}]"

    def __lt__(self, other: Waypoint) -> bool:
        return self.t < other.t