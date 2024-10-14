#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from __future__ import annotations
import commands2
import typing

from subsystems.drivetrain import Drivetrain
from wpimath.geometry import Rotation2d

class AimToDirectionConstants:
    kPRotate = 0.03
    kMinTurnSpeed = 0.2  # turning slower than this is unproductive
    kAngleToleranceDegrees = 5.0  # plus minus 5 degrees is ok

class AimToDirection(commands2.Command):
    def __init__(self, degrees: float | typing.Callable[[], float], drivetrain: Drivetrain, maxspeed: float = 1) -> None:
        self.targetDegrees = degrees
        self.maxSpeed = min((1.0, abs(maxspeed)))
        self.targetDirection = None
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

    def initialize(self):
        if callable(self.targetDegrees):
            self.targetDirection = Rotation2d.fromDegrees(self.targetDegrees())
        else:
            self.targetDirection = Rotation2d.fromDegrees(self.targetDegrees)

    def execute(self):
        # 1. how many degrees are left to turn?
        currentDirection = self.drivetrain.getHeading()
        rotationRemaining = self.targetDirection - currentDirection
        degreesRemaining = rotationRemaining.degrees()

        # 2. proportional control: if we are almost finished turning, use slower turn speed (to avoid overshooting)
        turnSpeed = AimToDirectionConstants.kPRotate * abs(degreesRemaining)
        if turnSpeed < AimToDirectionConstants.kMinTurnSpeed:
            turnSpeed = AimToDirectionConstants.kMinTurnSpeed  # but not too small
        if turnSpeed > self.maxSpeed:
            turnSpeed = self.maxSpeed  # and not above maxSpeed

        # 3. act on it! if target angle is on the right, turn right
        if degreesRemaining > 0:
            self.drivetrain.arcadeDrive(0.0, turnSpeed)
        else:
            self.drivetrain.arcadeDrive(0.0, -turnSpeed)  # otherwise, turn left

    def end(self, interrupted: bool):
        self.drivetrain.arcadeDrive(0, 0)

    def isFinished(self) -> bool:
        currentDirection = self.drivetrain.getHeading()
        rotationRemaining = self.targetDirection - currentDirection
        degreesRemaining = rotationRemaining.degrees()
        # if we are pretty close to the direction we wanted, consider the command finished
        if abs(degreesRemaining) < AimToDirectionConstants.kAngleToleranceDegrees:
            return True
