import pypathfinder as pf

#We are really only keeping track of 2 planes of reference
#1. The field: The plane the robot turns and moves on
#2. Robot Frame: The robot itself
#To do this we use Transforms that represent each plane
#However we only need 1 transform to represent the robot
#Field to Robot: Tracked over time with encoders and gyros. 
#Might drift over a match but we really only need this for auto

class RobotState:
    _instance = None

    MAX_OBSERVATION_BUFFER_SIZE = 100
    MAX_TARGET_AGE = 0.4

    @staticmethod
    def get_instance():
        if RobotState._instance is None:
            RobotState._instance = RobotState()
        return RobotState._instance

    def __init__(self):
        self.reset(0, pf.RigidTransform2D(pf.Translation2D(), pf.Rotation2D()))

    def reset(self, start_time, initial_field_to_robot: pf.RigidTransform2D):
        self._field_to_vehicle = pf.InterpolatingDict(self.MAX_OBSERVATION_BUFFER_SIZE)
        self._vehicle_velocity = pf.RigidTransform2D.Delta()

        self._field_to_vehicle.update({pf.InterpolatingValue(start_time): initial_field_to_robot})

    def get_field_to_vehicle(self, timestamp):
        return self._field_to_vehicle.get_interpolated(pf.InterpolatingValue(timestamp))

    def get_latest_field_to_vehicle(self):
        time, latest = self._field_to_vehicle.popitem(last=False)
        self._field_to_vehicle.update({time: latest})
        return latest

    def get_predicted_field_to_vehicle(self, lookahead_time):
        return self.get_latest_field_to_vehicle()[1].transformby(
                                                                    pf.RigidTransform2D.from_velocity(pf.RigidTransform2D.Delta(self._vehicle_velocity.dX * lookahead_time, 
                                                                                                                                self._vehicle_velocity.dY * lookahead_time, 
                                                                                                                                self._vehicle_velocity.dTheta * lookahead_time))
                                                                )
    
    def add_observation(self, timestamp, observation, velocity):
        self._field_to_vehicle.update({pf.InterpolatingValue(timestamp): observation})
        self._vehicle_velocity = velocity

    def generate_odometry_from_sensors(self, left_encoder_delta, right_encoder_delta, current_gyro):
        last_measurement = self.get_latest_field_to_vehicle()
        return pf.Kinematics.integrate_forward_kinematics(last_measurement, left_encoder_delta, right_encoder_delta, current_gyro)
