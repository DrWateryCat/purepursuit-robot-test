import wpilib
import magicbot
import planefollower
from components import drive, odometry

class Robot(magicbot.MagicRobot):
    drive = drive.Drive
    odometry = odometry.Odometry
    state = planefollower.RobotState.get_instance()
    def createObjects(self):
        self.left_joystick = wpilib.Joystick(0)
        self.right_joystick = wpilib.Joystick(1)
    def teleopInit(self):
        pass
    def teleopPeriodic(self):
        self.drive.set_open_loop((self.left_joystick.getRawAxis(1), self.right_joystick.getRawAxis(1)))

if __name__ == "__main__":
    wpilib.run(Robot)
