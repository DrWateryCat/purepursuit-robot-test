from planefollower import RobotState
from components import drive
import wpilib
import pypathfinder as pf

class Odometry:
    drive = drive.Drive
    def setup(self):
        self.robot_state = RobotState.get_instance()
        self.left_last_encoder = self.drive.get_left_distance_inches()
        self.right_last_encoder = self.drive.get_right_distance_inches()


    def execute(self):
        time = wpilib.Timer.getFPGATimestamp()
        left_distance = self.drive.get_left_distance_inches()
        right_distance = self.drive.get_right_distance_inches()
        gyro_angle = self.drive.get_gyro_angle()

        odometry = self.robot_state.generate_odometry_from_sensors(
                                                                    (left_distance - self.left_last_encoder),
                                                                    (right_distance - self.right_last_encoder),
                                                                    gyro_angle
                                                                  )

        velocity = pf.Kinematics.forward_kinematics(self.drive.get_left_velocity_inches_per_second(), self.drive.get_right_velocity_inches_per_second())
        self.robot_state.add_observation(time, odometry, velocity)

        self.left_last_encoder = left_distance
        self.right_last_encoder = right_distance