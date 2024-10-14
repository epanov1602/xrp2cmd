#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2

from subsystems.drivetrain import Drivetrain

class DriveDistance(commands2.Command):
    def __init__(self, speed: float, inches: float, drivetrain: Drivetrain) -> None:
        """Creates a new DriveDistance.
        This command will drive your robot for a desired distance at a desired speed.

        :param speed:  The speed at which the robot will drive, between 0.0 and 1.0
        :param inches: The number of inches the robot will drive
        :param drivetrain:  The drivetrain subsystem on which this command will run
        """
        super().__init__()

        self.distanceToTravel = inches
        self.speed = speed
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)
        self.startPoint = None

    def initialize(self) -> None:
        """Called when the command is initially scheduled."""
        self.drivetrain.arcadeDrive(0, 0)
        self.startPoint = self.drivetrain.getLocation()

    def execute(self) -> None:
        """Called every time the scheduler runs while the command is scheduled."""
        self.drivetrain.arcadeDrive(self.speed, 0)

    def end(self, interrupted: bool) -> None:
        """Called once the command ends or is interrupted."""
        self.drivetrain.arcadeDrive(0, 0)

    def isFinished(self) -> bool:
        """Returns true when the command should end."""
        # Compare distance travelled from start to desired distance
        currentPoint = self.drivetrain.getLocation()
        if currentPoint.distance(self.startPoint) >= self.distanceToTravel:
            return True
