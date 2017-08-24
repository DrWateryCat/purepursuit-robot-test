from robotpy_ext.autonomous import StatefulAutonomous, state
from components import drive
import pypathfinder as pf

class ForwardFiveFeet(StatefulAutonomous):
    MODE_NAME = "Forward Five Feet"

    drive = drive.Drive
    def initialize(self):
        pass

    @state(first=True)
    def start(self, tm, state_tm, initial_call):
        if initial_call:
            waypoints = [
                pf.Waypoint(pf.Translation2D(), 48.0),
                pf.Waypoint(pf.Translation2D(5, 0), 0)
            ]

            path = pf.Path(waypoints)
            self.drive.follow_path(path, False)
        if self.drive.finished_path():
            self.done()
