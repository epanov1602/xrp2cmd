import commands2
import typing

from commands.aimtodirection import AimToDirectionConstants

from subsystems.drivetrain import Drivetrain
from subsystems.cvcamera import CVCamera
from wpimath.geometry import Rotation2d

class AimToObject(commands2.Command):
    def __init__(self, camera: CVCamera, drivetrain: Drivetrain, speed=1.0, seek_speed=0.15) -> None:
        self.targetCamera = camera
        self.seekSpeed = seek_speed
        self.maxSpeed = min((1.0, abs(speed)))
        self.targetDirection = None
        self.minObjectIndex = None
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

    def initialize(self):
        self.targetDirection = None
        t, index, center, size = self.targetCamera.get_detected_object()
        self.minObjectIndex = index + 1  # newly detected objects must have index above this

    def execute(self):
        currentDirection = self.drivetrain.getHeading()

        # 0. if we don't know where to turn, look at the camera to see if any new object is detected
        if self.targetDirection is None:
            t, index, center, size = self.targetCamera.get_detected_object()
            if index <= self.minObjectIndex or center[0] is None:
                self.drivetrain.arcadeDrive(0, 0)
                return  # no new object detected, so stop and maybe look later
            # otherwise we have an object: compute how many degrees to turn towards that object
            self.targetDirection = currentDirection.rotateBy(Rotation2d.fromDegrees(-center[0]))

        # 1. how many degrees are left to turn?
        rotationRemaining = self.targetDirection - currentDirection
        degreesRemaining = rotationRemaining.degrees()

        # 2. proportional control: if we are almost finished turning, use slower turn speed (to avoid overshooting)
        turnSpeed = self.maxSpeed
        proportionalSpeed = AimToDirectionConstants.kP * abs(degreesRemaining)
        if turnSpeed > proportionalSpeed:
            turnSpeed = proportionalSpeed
        if turnSpeed < AimToDirectionConstants.kMinTurnSpeed:
            turnSpeed = AimToDirectionConstants.kMinTurnSpeed  # but not too small

        # 3. act on it! if target angle is on the right, turn right
        if degreesRemaining > 0:
            self.drivetrain.arcadeDrive(0.0, turnSpeed)
            print(f"AimToDirection: {degreesRemaining} degrees remaining, {turnSpeed} turn speed")
        else:
            self.drivetrain.arcadeDrive(0.0, -turnSpeed)  # otherwise, turn left
            print(f"AimToDirection: {degreesRemaining} degrees remaining, {-turnSpeed} turn speed")

    def end(self, interrupted: bool):
        self.drivetrain.arcadeDrive(0, 0)

    def isFinished(self) -> bool:
        if self.targetDirection is None:
            return False  # not finished yet
        currentDirection = self.drivetrain.getHeading()
        rotationRemaining = self.targetDirection - currentDirection
        degreesRemaining = rotationRemaining.degrees()
        # if we are pretty close to the direction we wanted, consider the command finished
        if abs(degreesRemaining) < AimToDirectionConstants.kAngleToleranceDegrees:
            turnVelocity = self.drivetrain.getGyroVelocityZ()
            print(f"AimToDirection: possible stopping velocity {turnVelocity}")
            if abs(turnVelocity) < AimToDirectionConstants.kAngleVelocityToleranceDegreesPerSec:
                print(f"AimToDirection: finished with velocity {turnVelocity}")
                return True
