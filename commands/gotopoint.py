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
from wpilib import SmartDashboard

class GoToPointConstants:
    kPTranslate = 0.04
    kMinTranslateSpeed = 0.3  # moving forward slower than this is unproductive
    kOversteerAdjustment = 0.5

class GoToPoint(commands2.Command):
    def __init__(self, x, y, drivetrain, speed=1.0, slowDownAtFinish=True) -> None:
        self.targetPosition = Translation2d(x, y)
        self.speed = speed
        self.stop = slowDownAtFinish
        self.initialDirection = None
        self.initialDistance = None
        self.pointingInGoodDirection = False
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

    def initialize(self):
        self.initialPosition = self.drivetrain.getPose().translation()
        initialDirection = self.targetPosition - self.initialPosition
        self.initialDirection = Rotation2d(initialDirection.x, initialDirection.y)
        self.initialDistance = self.initialPosition.distance(self.targetPosition)
        self.pointingInGoodDirection = False

    def execute(self):
        # 1. to which direction we should be pointing?
        currentPose = self.drivetrain.getPose()
        currentDirection = currentPose.rotation()
        currentPoint = currentPose.translation()
        targetDirectionVector = self.targetPosition - currentPoint
        targetDirection = Rotation2d(targetDirectionVector.x, targetDirectionVector.y)
        degreesRemaining = (targetDirection - currentDirection).degrees()

        # 2. if we are pointing in a very wrong direction (more than 45 degrees away), rotate away without moving
        if degreesRemaining > 45 and not self.pointingInGoodDirection:
            self.drivetrain.arcadeDrive(0.0, self.speed)
            return
        elif degreesRemaining < -45 and not self.pointingInGoodDirection:
            self.drivetrain.arcadeDrive(0.0, self.speed)
            return
        else:
            self.pointingInGoodDirection = True

        # 3. otherwise, drive forward but with an oversteer adjustment
        if GoToPointConstants.kOversteerAdjustment != 0:
            deviationFromInitial = (targetDirection - self.initialDirection).degrees()
            adjustment = GoToPointConstants.kOversteerAdjustment * deviationFromInitial
            if adjustment > 20: adjustment = 20  # avoid oscillations by capping the adjustment at 20 degrees
            if adjustment < -20: adjustment = -20  # avoid oscillations by capping the adjustment at 20 degrees
            targetDirection = targetDirection.rotateBy(Rotation2d.fromDegrees(adjustment))
            degreesRemaining = (targetDirection - currentDirection).degrees()

        SmartDashboard.putNumber("z-heading-target", targetDirection.degrees())

        # 3. now when we know the desired direction, we can compute the turn speed
        rotateSpeed = abs(self.speed)
        proportionalRotateSpeed = AimToDirectionConstants.kP * abs(degreesRemaining)
        if rotateSpeed > proportionalRotateSpeed:
            rotateSpeed = proportionalRotateSpeed

        # 5. but if not too different, then we can drive while turning
        distanceRemaining = self.targetPosition.distance(currentPoint)
        proportionalTransSpeed = GoToPointConstants.kPTranslate * distanceRemaining
        translateSpeed = self.speed  # if we don't plan to stop at the end, go at max speed
        if translateSpeed > proportionalTransSpeed and self.stop:
            translateSpeed = proportionalTransSpeed
        if translateSpeed < GoToPointConstants.kMinTranslateSpeed:
            translateSpeed = GoToPointConstants.kMinTranslateSpeed

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

        # 1. have we reached the point where we are moving very slowly?
        tooSlowNow = translateSpeed < 0.25 * GoToPointConstants.kMinTranslateSpeed and self.stop

        # 2. did we overshoot?
        distanceFromInitialPosition = self.initialPosition.distance(currentPosition)
        if distanceFromInitialPosition >= self.initialDistance or tooSlowNow:
            SmartDashboard.putNumber("distance-to-target", self.targetPosition.distance(currentPosition))
            return True  # we overshot
