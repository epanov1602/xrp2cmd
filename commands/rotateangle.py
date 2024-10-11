#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2

from subsystems.drivetrain import Drivetrain
import wpilib

class RotateAngle(commands2.Command):
    def __init__(self, speed: float, degrees: float, drivetrain: Drivetrain) -> None:
        """Creates a new DriveDistance.
        This command will drive your robot for a desired distance at a desired speed.

        :param speed:  The speed at which to turn (between -1.0 and 1.0 but not zero)
        :param degrees: The number of degrees the robot will turn, less than 180
        :param drivetrain:  The drivetrain subsystem on which this command will run
        """
        super().__init__()

        assert degrees > 0, "only positive turning angles are supported (you can use negative speeds)"
        assert degrees < 135, "only values under 135 degrees are supported"
        self.degreesToTurn = degrees

        self.speed = speed
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)
        self.startHeading = None

    def initialize(self) -> None:
        """Called when the command is initially scheduled."""
        self.drivetrain.arcadeDrive(0, 0)
        self.startHeading = self.drivetrain.getHeading()

    def execute(self) -> None:
        """Called every time the scheduler runs while the command is scheduled."""
        self.drivetrain.arcadeDrive(0, self.speed)

    def end(self, interrupted: bool) -> None:
        """Called once the command ends or is interrupted."""
        self.drivetrain.arcadeDrive(0, 0)

    def isFinished(self) -> bool:
        """Returns true when the command should end."""
        # Compare distance travelled from start to desired distance
        currentHeading = self.drivetrain.getHeading()
        if abs((currentHeading - self.startHeading).degrees()) >= self.degreesToTurn:
            return True
