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
    kPRotate = 0.002  # 0.002 is the default
    kMinTurnSpeed = 0.15  # turning slower than this is unproductive for the motor (might not even spin)
    kAngleToleranceDegrees = 3.0  # plus minus 3 degrees is "close enough" (for a cheap XRP robot)
    kAngleVelocityToleranceDegreesPerSec = 50  # velocity under 100 degrees/second is considered "stopped"


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
        turnSpeed = self.maxSpeed
        proportionalSpeed = AimToDirectionConstants.kPRotate * abs(degreesRemaining)
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
