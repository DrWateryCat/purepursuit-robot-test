class Config:
    class Drive:
        DRIVE_VELOCITY_P = 1
        DRIVE_VELOCITY_I = 0
        DRIVE_VELOCITY_D = 0
        DRIVE_VELOCITY_F = 0
        DRIVE_VELOCITY_I_ZONE = 0
        DRIVE_VELOCITY_RAMP_RATE = 0
        DRIVE_VELOCITY_ALLOWABLE_ERROR = 0.25

        DRIVE_WHEEL_DIAMETER = 6
        DRIVE_WHEEL_SLIP_FACTOR = 0.5

    class PathFollowing:
        PATH_FOLLOWING_LOOKAHEAD = 24
        PATH_FOLLOWING_MAX_VELOCITY = 120
        PATH_FOLLOWING_MAX_ACCELERATION = 80

    class DriveHeading:
        VELOCITY_P = 1
        VELOCITY_I = 0
        VELOCITY_D = 0