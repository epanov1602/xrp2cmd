import commands2
from wpilib import SmartDashboard, Timer


class Stopwatch(commands2.Subsystem):
    def __init__(self, name: str):
        super().__init__()
        self.started = None
        self.name = name

    def periodic(self):
        if self.start is not None:
            elapsed = Timer.getFPGATimestamp() - self.started
            SmartDashboard.putNumber(self.name, 0)

    def start(self):
        self.started = Timer.getFPGATimestamp()

    def stop(self):
        self.periodic()
        self.started = None
