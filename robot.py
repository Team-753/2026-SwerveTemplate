import robotpy,commands2,wpilib
from DriveTrain.drivetrainCommand import driveTrainCommand,driveTrainSubsystem
class myRobot(commands2.TimedCommandRobot):
    def __init__(self):
        pass
        self.drivetrainSubsys=driveTrainSubsystem()
        self.drivetrainSubsys.setDefaultCommand(driveTrainCommand(self.drivetrainSubsys))
        
    def teleopPeriodic(self):
        return super().teleopPeriodic()