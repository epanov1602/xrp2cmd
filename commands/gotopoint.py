#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from __future__ import annotations
import commands2
import typing

from subsystems.drivetrain import Drivetrain
from commands.aimtodirection import AimToDirectionConstants
from wpimath.geometry import Rotation2d, Translation2d

class GoToPointConstants:
    kPTranslate = 0.04
    kMinTranslateSpeed = 0.3  # moving forward slower than this is unproductive
    kOversteerAdjustment = 0.5

class GoToPoint(commands2.Command):
    def __init__(self, x: float, y: float, drivetrain: Drivetrain, stopAtEnd: bool = True) -> None:
        self.targetPosition = Translation2d(x, y)
        self.stopAtEnd = stopAtEnd
        self.initialDirection = None
        self.initialDistance = None
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

    def initialize(self):
        self.initialPosition = self.drivetrain.getPose().translation()
        initialDirection = self.targetPosition - self.initialPosition
        self.initialDirection = Rotation2d(initialDirection.x, initialDirection.y)
        self.initialDistance = self.initialPosition.distance(self.targetPosition)

    def execute(self):
        # 1. to which direction we should be pointing?
        currentPose = self.drivetrain.getPose()
        currentDirection = currentPose.rotation()
        currentPoint = currentPose.translation()
        targetDirection = self.targetPosition - currentPoint
        targetDirection = Rotation2d(targetDirection.x, targetDirection.y)
        degreesRemaining = (targetDirection - currentDirection).degrees()

        # 2. apply the oversteer adjustment to the direction?
        if abs(degreesRemaining) < 45 and GoToPointConstants.kOversteerAdjustment != 0:
            deviationFromInitial = (targetDirection - self.initialDirection).degrees()
            adjustment = Rotation2d.fromDegrees(GoToPointConstants.kOversteerAdjustment * deviationFromInitial)
            targetDirection = targetDirection.rotateBy(adjustment)
            degreesRemaining = (targetDirection - currentDirection).degrees()

        # 3. now when we know the desired direction, we can compute the turn speed
        rotateSpeed = AimToDirectionConstants.kPRotate * abs(degreesRemaining)
        if rotateSpeed < AimToDirectionConstants.kMinTurnSpeed:
            rotateSpeed = AimToDirectionConstants.kMinTurnSpeed  # but not too small
        if rotateSpeed > 1:
            rotateSpeed = 1  # and not above 1.0

        # 4. if we are pointing in a very different direction, turn towards where we need to be without driving towards
        if degreesRemaining > 60:
            self.drivetrain.arcadeDrive(0.0, rotateSpeed)
            return
        elif degreesRemaining < -60:
            self.drivetrain.arcadeDrive(0.0, -rotateSpeed)
            return

        # 5. but if not too different, then we can drive while turning
        distanceRemaining = self.targetPosition.distance(currentPoint)
        if not self.stopAtEnd:
            translateSpeed = 1  # if we don't plan to stop at the end, go at max speed
        else:
            translateSpeed = GoToPointConstants.kPTranslate * distanceRemaining  # else, proportional
        if translateSpeed < GoToPointConstants.kMinTranslateSpeed:
            translateSpeed = GoToPointConstants.kMinTranslateSpeed
        if translateSpeed > 1:
            translateSpeed = 1

        # 6. if we need to be turning left while driving, use negative rotation speed
        if degreesRemaining < 0:
            self.drivetrain.arcadeDrive(translateSpeed, -rotateSpeed)
        else:  # otherwise, use positive
            self.drivetrain.arcadeDrive(translateSpeed, rotateSpeed)

    def end(self, interrupted: bool):
        self.drivetrain.arcadeDrive(0, 0)

    def isFinished(self) -> bool:
        # 1. did we reach the point where we must move very slow?
        currentPose = self.drivetrain.getPose()
        currentPosition = currentPose.translation()
        distanceRemaining = self.targetPosition.distance(currentPosition)
        translateSpeed = GoToPointConstants.kPTranslate * distanceRemaining
        if translateSpeed < 0.5 * GoToPointConstants.kMinTranslateSpeed:
            return True  # we reached the point where we are moving very slow, time to stop

        # 2. did we overshoot?
        distanceFromInitialPosition = self.initialPosition.distance(currentPosition)
        if distanceFromInitialPosition >= self.initialDistance:
            return True  # we overshot
