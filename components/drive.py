import wpilib
import ctre
import magicbot
import pypathfinder as pf
import math
from config import Config
from planefollower import RobotState
from robotpy_ext.common_drivers import navx

import logging
log = logging.getLogger("Drive")

class Drive:
    class _DriveState:
        OPEN_LOOP = 0
        PATH_FOLLOWING = 1
        VELOCITY_SETPOINT = 2

    def setup(self):
        self.left_master = ctre.CANTalon(0)
        self.left_slave = ctre.CANTalon(1)
        self.right_master = ctre.CANTalon(2)
        self.right_slave = ctre.CANTalon(3)

        self.gyro = navx.AHRS.create_spi()

        self.current_state = self._DriveState.OPEN_LOOP

        self.left_slave.changeControlMode(ctre.CANTalon.ControlMode.Follower)
        self.left_slave.set(self.left_master.getDeviceID())

        self.right_slave.changeControlMode(ctre.CANTalon.ControlMode.Follower)
        self.right_slave.set(self.right_master.getDeviceID())

        self.left_master.changeControlMode(ctre.CANTalon.ControlMode.PercentVbus)
        self.left_master.set(0)

        self.right_master.changeControlMode(ctre.CANTalon.ControlMode.PercentVbus)
        self.right_master.set(0)

        self.left_master.setPID(Config.Drive.DRIVE_VELOCITY_P, 
                                Config.Drive.DRIVE_VELOCITY_I, 
                                Config.Drive.DRIVE_VELOCITY_D,
                                Config.Drive.DRIVE_VELOCITY_F,
                                Config.Drive.DRIVE_VELOCITY_I_ZONE,
                                Config.Drive.DRIVE_VELOCITY_RAMP_RATE,
                                0)

        self.right_master.setPID(Config.Drive.DRIVE_VELOCITY_P, 
                                Config.Drive.DRIVE_VELOCITY_I, 
                                Config.Drive.DRIVE_VELOCITY_D,
                                Config.Drive.DRIVE_VELOCITY_F,
                                Config.Drive.DRIVE_VELOCITY_I_ZONE,
                                Config.Drive.DRIVE_VELOCITY_RAMP_RATE,
                                0)

        self.left_master.setFeedbackDevice(ctre.CANTalon.FeedbackDevice.QuadEncoder)
        self.left_master.reverseSensor(True)

        self.right_master.setFeedbackDevice(ctre.CANTalon.FeedbackDevice.QuadEncoder)
        self.right_master.reverseSensor(False)
        self.right_master.reverseOutput(True)

        self.left_value = 0
        self.right_value = 0

    def rotations_to_inches(self, rots):
        return rots * (Config.Drive.DRIVE_WHEEL_DIAMETER * math.pi)

    def rpm_to_inches_per_second(self, rpm):
        return self.rotations_to_inches(rpm) / 60

    def inches_to_rotations(self, inches):
        return inches / (Config.Drive.DRIVE_WHEEL_DIAMETER * math.pi)

    def inches_per_second_to_rpm(self, ips):
        return self.inches_to_rotations(ips) / 60

    def get_left_distance_inches(self):
        return self.rotations_to_inches(self.left_master.getPosition())

    def get_right_distance_inches(self):
        return self.rotations_to_inches(self.right_master.getPosition())

    def get_left_velocity_inches_per_second(self):
        return self.rpm_to_inches_per_second(self.left_master.getSpeed())

    def get_right_velocity_inches_per_second(self):
        return self.rpm_to_inches_per_second(self.right_master.getPosition())

    def get_gyro(self):
        return self.gyro

    def get_gyro_angle(self):
        return pf.Rotation2D.from_degrees(self.gyro.getYaw())

    def set_left_right(self, left, right):
        self.left_value = left
        self.right_value = right

    def set_open_loop(self, drive_signal):
        if self.current_state is not self._DriveState.OPEN_LOOP:
            self.left_master.changeControlMode(ctre.CANTalon.ControlMode.PercentVbus)
            self.right_master.changeControlMode(ctre.CANTalon.ControlMode.PercentVbus)
        self.set_left_right(drive_signal[0], drive_signal[1])

    def configure_talons_for_speed_control(self):
        if self.current_state is self._DriveState.OPEN_LOOP:
            self.left_master.changeControlMode(ctre.CANTalon.ControlMode.Speed)
            self.left_master.setProfile(0)
            self.left_master.setAllowableClosedLoopErr(Config.Drive.DRIVE_VELOCITY_ALLOWABLE_ERROR)

            self.right_master.changeControlMode(ctre.CANTalon.ControlMode.Speed)
            self.right_master.setProfile(0)
            self.right_master.setAllowableClosedLoopErr(Config.Drive.DRIVE_VELOCITY_ALLOWABLE_ERROR)

    def update_velocity_setpoint(self, drive_data):
        if self.current_state is not self._DriveState.OPEN_LOOP:
            self.left_value = drive_data.left
            self.right_value = drive_data.right
        else:
            log.warn("Hit a bad state! Fix your code!")
            self.left_value = 0
            self.right_value = 0

    def update_path_follower(self):
        current_pose = RobotState.get_instance().get_latest_field_to_vehicle()
        command = self.path_follower.update(current_pose, wpilib.Timer.getFPGATimestamp())
        setpoint = pf.Kinematics.inverse_kinematics(command, Config.Drive.DRIVE_WHEEL_SLIP_FACTOR, Config.Drive.DRIVE_WHEEL_DIAMETER)

        max_vel = 0
        max_vel = max(max_vel, abs(setpoint.left))
        max_vel = max(max_vel, abs(setpoint.right))

        if max_vel > Config.PathFollowing.PATH_FOLLOWING_MAX_VELOCITY:
            scaling = Config.PathFollowing.PATH_FOLLOWING_MAX_VELOCITY / max_vel
            setpoint = pf.Kinematics.DriveData(setpoint.left * scaling, setpoint.right * scaling)

        self.update_velocity_setpoint(setpoint)


    def follow_path(self, path, reverse):
        if self.current_state is not self._DriveState.PATH_FOLLOWING:
            self.configure_talons_for_speed_control()
            self.current_state = self._DriveState.PATH_FOLLOWING

        self.path_follower = pf.AdaptivePurePursuitController(Config.PathFollowing.PATH_FOLLOWING_LOOKAHEAD,
                                                              Config.PathFollowing.PATH_FOLLOWING_MAX_ACCELERATION,
                                                              20,
                                                              path,
                                                              reverse,
                                                              0.25)
        self.update_path_follower()

    def finished_path(self):
        return (self.current_state is self._DriveState.PATH_FOLLOWING and self.path_follower.is_done()) or self.current_state is not self._DriveState.PATH_FOLLOWING

    def get_markers_crossed(self):
        if self.current_state is not self._DriveState.PATH_FOLLOWING:
            return None
        else:
            return self.path_follower.get_markers_crossed()

    def stop(self):
        self.set_left_right(0, 0)

    def execute(self):
        if self.current_state is self._DriveState.PATH_FOLLOWING:
            self.update_path_follower()
            if self.finished_path():
                self.stop()

        self.left_master.set(self.left_value)
        self.right_master.set(self.right_value)

        self.left_value = 0
        self.right_value = 0