#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import xrp


class Arm(commands2.Subsystem):
    def __init__(self, initialAngle=90) -> None:
        super().__init__()
        # Device number 4 maps to the physical Servo 1 port on the XRP
        self.armServo = xrp.XRPServo(4)
        self.armServo.setAngle(initialAngle)

    def setAngle(self, degrees: float):
        self.armServo.setAngle(degrees)
