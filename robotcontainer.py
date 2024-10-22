#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import typing

import wpilib
import commands2
from commands2.button import CommandXboxController
from commands2 import InstantCommand

from commands.arcadedrive import ArcadeDrive
from commands.drivedistance import DriveDistance
from commands.rotateangle import RotateAngle

from subsystems.drivetrain import Drivetrain
from subsystems.arm import Arm
from subsystems.stopwatch import Stopwatch

from commands.aimtodirection import AimToDirection
from commands.gotopoint import GoToPoint

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self):
        # The robot's subsystems are defined here
        self.drivetrain = Drivetrain()
        self.arm = Arm()
        self.stopwatch = Stopwatch("race-time")

        # Assume that joystick "j0" is plugged into channnel 0
        self.j0 = CommandXboxController(0)
        # (you can also use CommandPS4Controller or CommandJoystick, if you prefer those)

        self.configureButtonBindings()

    def configureButtonBindings(self):
        """Use this method to define your button->command mappings"""

        # 1. Here is a command to drive forward 10 inches with speed 0.9
        forward10inches = DriveDistance(speed=0.9, inches=10, drivetrain=self.drivetrain)
        # let's bind this command to button "y" on the joystick
        self.j0.y().onTrue(forward10inches)

        # and here is a command to drive back 10 inches
        back10inches = DriveDistance(speed=-0.7, inches=10, drivetrain=self.drivetrain)

        #  - exercise 1: can you hook this command to button "a" on the joystick?


        # 3. Instant commands (commands that just do one thing instantly)
        # normally, simple one-shot commands that don't need to be written as separate modules in commands/ directory

        # "lambda" really means "do this later when that command needs to run"
        arm_up = commands2.InstantCommand(lambda: self.arm.setAngle(90))
        self.j0.x().onTrue(arm_up)  # - bind it to button "x" pressed

        # a command for "arm down" can bind to button x "unpressed" ???
        arm_down = commands2.InstantCommand(lambda: self.arm.setAngle(0))
        # yes! we can bind it to "button unpressed" event, if we use "onFalse()"
        self.j0.x().onFalse(arm_down)

        #  - exercise 3: can you make an "arm half up" (45 degrees) command and bind it to "B button pressed"?

        # (and is anything missing?)


        # 4. A command to turn right 45 degrees *but* we can add a 5 second timeout to it
        right45degrees = RotateAngle(speed=0.6, degrees=+45, drivetrain=self.drivetrain)
        right45degrees_timeout5s = right45degrees.withTimeout(5)
        self.j0.rightBumper().onTrue(right45degrees_timeout5s)

        # exercise 4: can you make a command to turn the robot left by 45 degrees and with 3 second timeout?

        # exercise 4b: can you bind this command to the left bumper button of the joystick?


        # 5. Connecting commands together: making a half square
        forward8inches1 = DriveDistance(speed=0.7, inches=8, drivetrain=self.drivetrain)
        right90degrees1 = RotateAngle(speed=0.5, degrees=90, drivetrain=self.drivetrain)
        forward8inches2 = DriveDistance(speed=0.7, inches=8, drivetrain=self.drivetrain)
        right90degrees2 = RotateAngle(speed=0.5, degrees=90, drivetrain=self.drivetrain)
        half_square = forward8inches1.andThen(right90degrees1).andThen(forward8inches2).andThen(right90degrees2)
        self.j0.povDown().onTrue(half_square)

        # exercise 5: can you actually change the code above to make it a full square?


        # 6. A little helper instant command to reset the robot coordinates in SmartDashboard
        reset_coordinates = commands2.InstantCommand(lambda: self.drivetrain.resetOdometry())
        self.j0.povUp().onTrue(reset_coordinates)

        # 7. Finally, a command to take input from joystick *later* ("lambda" = later)
        # and drive using that input as control speed signal
        drive = ArcadeDrive(
            self.drivetrain,
            lambda: -self.j0.getRawAxis(1),  # minus sign, because Xbox stick pushed forward is negative axis value
            lambda: -self.j0.getRawAxis(0),
        )
        # This command will be running *by default* on drivetrain
        # ("by default" means it will stop running when some other command is asked
        # to use drivetrain, and will restart running after that other command is done)
        self.drivetrain.setDefaultCommand(drive)

    def getAutonomousCommand(self):
        # - exercise B1: can you make a command to drive 20 inches forward at max speed?
        startStopwatch = InstantCommand(self.stopwatch.start)
        goTo40inch = GoToPoint(40, 0, self.drivetrain)
        stopStopwatch = InstantCommand(self.stopwatch.stop)

        # - exercise B2: can you return this command instead of None?
        autoCommand = startStopwatch.andThen(goTo40inch).andThen(stopStopwatch)
        return autoCommand

    def teleopInit(self):
        self.drivetrain.resetOdometry()
