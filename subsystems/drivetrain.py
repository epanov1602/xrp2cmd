#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math

import commands2
import wpilib
import xrp

from wpilib.drive import DifferentialDrive
from wpimath.kinematics import DifferentialDriveOdometry
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpilib import SmartDashboard


class Drivetrain(commands2.Subsystem):
    kCountsPerRevolution = 585.0
    kWheelDiameterInch = 2.3622

    def __init__(self) -> None:
        super().__init__()

        # The XRP has the left and right motors set to
        # PWM channels 0 and 1 respectively
        self.leftMotor = xrp.XRPMotor(0)
        self.rightMotor = xrp.XRPMotor(1)
        self.rightMotor.setInverted(True)

        # The XRP has onboard encoders that are hardcoded
        # to use DIO pins 4/5 and 6/7 for the left and right
        self.leftEncoder = wpilib.Encoder(4, 5)
        self.rightEncoder = wpilib.Encoder(6, 7)

        # And an onboard gyro (and you have to power on your XRP when it is on flat surface)
        self.gyro = xrp.XRPGyro()
        self.accelerometer = wpilib.BuiltInAccelerometer()
        self.reflectanceSensor = xrp.XRPReflectanceSensor()
        self.distanceSensor = xrp.XRPRangefinder()

        # Use inches as unit for encoder distances
        self.leftEncoder.setDistancePerPulse(
            (math.pi * self.kWheelDiameterInch) / self.kCountsPerRevolution
        )
        self.rightEncoder.setDistancePerPulse(
            (math.pi * self.kWheelDiameterInch) / self.kCountsPerRevolution
        )
        self.resetEncoders()
        self.resetGyro()

        # Set up the differential drive controller and differential drive odometry
        self.drive = DifferentialDrive(self.leftMotor, self.rightMotor)
        self.odometry = DifferentialDriveOdometry(
            Rotation2d.fromDegrees(self.getGyroAngleZ()), self.getLeftDistanceInch(), self.getRightDistanceInch())

    def periodic(self) -> None:
        heading = Rotation2d.fromDegrees(self.getGyroAngleZ())
        pose = self.odometry.update(heading, self.getLeftDistanceInch(), self.getRightDistanceInch())
        SmartDashboard.putNumber("distance-to-obst", self.getDistanceToObstacle())
        SmartDashboard.putNumber("left-reflect", self.reflectanceSensor.getLeftReflectanceValue())
        SmartDashboard.putNumber("right-reflect", self.reflectanceSensor.getRightReflectanceValue())
        SmartDashboard.putNumber("x", pose.x)
        SmartDashboard.putNumber("y", pose.y)
        SmartDashboard.putNumber("z-heading", pose.rotation().degrees())


    def arcadeDrive(self, fwd: float, rot: float) -> None:
        """
        Drives the robot using arcade controls.

        :param fwd: the commanded forward movement
        :param rot: the commanded rotation
        """
        self.drive.arcadeDrive(fwd, -rot)

    def stop(self) -> None:
        """
        Stop the drivetrain motors
        """
        self.drive.arcadeDrive(0, 0)

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        self.leftEncoder.reset()
        self.rightEncoder.reset()

    def getLeftEncoderCount(self) -> int:
        return self.leftEncoder.get()

    def getRightEncoderCount(self) -> int:
        return self.rightEncoder.get()

    def getLeftDistanceInch(self) -> float:
        return self.leftEncoder.getDistance()

    def getRightDistanceInch(self) -> float:
        return self.rightEncoder.getDistance()

    def getAverageDistanceInch(self) -> float:
        """Gets the average distance of the TWO encoders."""
        return (self.getLeftDistanceInch() + self.getRightDistanceInch()) / 2.0

    def getAccelX(self) -> float:
        """The acceleration in the X-axis.

        :returns: The acceleration of the XRP along the X-axis in Gs
        """
        return self.accelerometer.getX()

    def getAccelY(self) -> float:
        """The acceleration in the Y-axis.

        :returns: The acceleration of the XRP along the Y-axis in Gs
        """
        return self.accelerometer.getY()

    def getAccelZ(self) -> float:
        """The acceleration in the Z-axis.

        :returns: The acceleration of the XRP along the Z-axis in Gs
        """
        return self.accelerometer.getZ()

    def getGyroAngleX(self) -> float:
        """Current angle of the XRP around the X-axis.

        :returns: The current angle of the XRP in degrees
        """
        return self.gyro.getAngleX()

    def getGyroAngleY(self) -> float:
        """Current angle of the XRP around the Y-axis.

        :returns: The current angle of the XRP in degrees
        """
        return self.gyro.getAngleY()

    def getGyroAngleZ(self) -> float:
        """Current angle of the XRP around the Z-axis.

        :returns: The current angle of the XRP in degrees
        """
        return self.gyro.getAngleZ()

    def getDistanceToObstacle(self) -> float:
        """Distance to obstacle in the front, as given by the distance sensor

        :returns: Distance in meters, values >0.5 are not very reliable and are replaced with nan.
        """
        distance = self.distanceSensor.getDistance()
        return distance if distance < 0.5 else math.nan

    def getPose(self) -> Pose2d:
        return self.odometry.getPose()

    def getLocation(self) -> Translation2d:
        return self.getPose().translation()

    def getHeading(self) -> Rotation2d:
        return self.getPose().rotation()

    def resetGyro(self) -> None:
        """Reset the gyro"""
        self.gyro.reset()

    def resetOdometry(self, pose: Pose2d = Pose2d()) -> None:
        self.resetGyro()
        self.resetEncoders()
        heading = Rotation2d.fromDegrees(self.getGyroAngleZ())
        self.odometry.resetPosition(heading, self.getLeftDistanceInch(), self.getRightDistanceInch(), pose)

    def resetPose(self, pose: Pose2d = Pose2d()) -> None:
        self.resetOdometry(pose)
