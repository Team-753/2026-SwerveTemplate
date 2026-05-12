import commands2,phoenix6
class swerveModule():
    def __init__(self,driveMotorID,turnMotorID,canBusName:str="rio") -> None:
        self.driveMotor=phoenix6.hardware.TalonFX(driveMotorID,canBusName)
        self.turnMotor=phoenix6.hardware.TalonFX(turnMotorID,canBusName)
        driveMotorConfig=phoenix6.configs.TalonFXConfiguration()
        turnMotorConfig=phoenix6.configs.TalonFXConfiguration()
        self.goToPosition=phoenix6.controls.PositionVoltage()
    def setZero(self,rotations):
        self.turnMotor.set_position(rotations)
    def setState(self,swerveRotations,driveSpeedDutyCycle) -> None:
        self.driveMotor.set(driveSpeedDutyCycle)
        self.turnMotor.set_control(self.goToPosition.with_position(swerveRotations))

class driveTrainSubsystem(commands2.Subsystem):
    def __init__(self):
        self.swerveModules=[]
        self.swerveModules[0]=swerveModule(1,2)
        self.swerveModules[1]=swerveModule(3,4)
        self.swerveModules[2]=swerveModule(5,6)
        self.swerveModules[3]=swerveModule(7,8)
    def setDrivetrain(self,allSwerveSpeed,allSwerveRotation):   
        for i in range(4):
            self.swerveModules[i].setState(allSwerveRotation,allSwerveSpeed)

class driveTrainCommand(commands2.Command):
    def __init__(self,driveSubsys:driveTrainSubsystem):
        self.addRequirements(driveSubsys)
        self.driveSubsys=driveSubsys
        return super().initialize()
    def execute(self):
        self.driveSubsys.setDrivetrain(0,1)