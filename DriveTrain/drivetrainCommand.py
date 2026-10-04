import commands2,phoenix6,wpimath.kinematics
import drivetrainConfiguration as dtConfig
#the "swerveModule" class is a repeatable piece of code that you can name, if you're confused it might help to look at the "driveTrainSubsystem" class and see how its used
class swerveModule():
    def __init__(self,driveMotorID,turnMotorID,canBusName:str="rio") -> None:
                                                        #adding the ":str="rio" " makes it so the canBusName defaults to that value if nothing is put in
                                                        #adding "-> None" is just a standard procedure but it doesnt actually do anything
        #Defining a drive motor
        self.driveMotor=phoenix6.hardware.TalonFX(driveMotorID,canBusName)
        #Defining a turn motor
        self.turnMotor=phoenix6.hardware.TalonFX(turnMotorID,canBusName)

        #Defining a configuration for a drive motor NOTE Advanced: can be moved to drive train subsystem and then passed into this class
        driveMotorConfig=phoenix6.configs.TalonFXConfiguration()
        #Defining PID parameters (if you dont know what a PID controller is google it)
        driveMotorConfig.slot0.k_p = 0.1
        driveMotorConfig.slot0.k_i = 0
        driveMotorConfig.slot0.k_d = 0
        #Some coeffiecients to overcome motor friction and stuff
        driveMotorConfig.slot0.k_s = 0
        driveMotorConfig.slot0.k_v = 0
        #Apply configuration to motor
        self.driveMotor.configurator.apply(driveMotorConfig)

        #Defining a configuration for a turn motor NOTE Advanced: can be moved to turn train subsystem and then passed into this class
        turnMotorConfig=phoenix6.configs.TalonFXConfiguration()
        #Defining PID parameters (if you dont know what a PID controller is google it)
        turnMotorConfig.slot0.k_p = 0.1
        turnMotorConfig.slot0.k_i = 0
        turnMotorConfig.slot0.k_d = 0
        #Some coeffiecients to overcome motor friction and stuff
        turnMotorConfig.slot0.k_s = 0
        turnMotorConfig.slot0.k_v = 0
        #Apply configuration to motor
        self.turnMotor.configurator.apply(turnMotorConfig)

        #Creating some ways to control our motors like Position and Velocity
        self.goToPosition=phoenix6.controls.PositionVoltage()
        self.goToSpeed=phoenix6.controls.VelocityDutyCycle()
    def setZero(self,rotations):
        #Create a motor offset, it basically sets the current rotation of the motor as a certain value, so if its 90 degrees off instead of starting at 0 you can say you are at 90 degrees
        self.turnMotor.set_position(rotations)
    def setState(self,swerveRotations,driveSpeedMS) -> None:
        #Function that allows speeds and positions to be applied to the swerve module
        self.driveMotor.set_control(self.goToSpeed.with_velocity(driveSpeedMS/dtConfig.wheelCircumferenceMeters))
        self.turnMotor.set_control(self.goToPosition.with_position(swerveRotations))

class kinematicsHandlerClass():
    def __init__(self):
        wheel=dtConfig.wheel
        self.swerveKinematics=wpimath.kinematics.SwerveDrive4Kinematics(wheel["fr"],wheel["fl"],wheel["br"],wheel["bl"])
    def calculateKinematics(self,x,y,radPS,compassRot:wpimath.geometry.Rotation2d):
        #"compassRot:wpimath..." makes it so the compassRot variable defaults to being the Rotation2d
        swerveDesiredState=self.swerveKinematics.toSwerveModuleStates(wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(x,y,radPS,compassRot))
        return swerveDesiredState

class driveTrainSubsystem(commands2.Subsystem):
    def __init__(self):
        self.compass=phoenix6.hardware.Pigeon2(9)
        #Creating a list of all the swerve modules and their respective motor ID's
        self.swerveModules: list[swerveModule]=[]
        #This "self.swerveModule: list[swerveModule]=[]" type hints the self.swerveModules list so that in "setDrivetrain" 
        #the list in the "for" loop will have the swerve module type,
        #see what happens if you comment the wierd part out if you dont understand
        self.swerveModules[0]=swerveModule(1,2)
        self.swerveModules[1]=swerveModule(3,4)
        self.swerveModules[2]=swerveModule(5,6)
        self.swerveModules[3]=swerveModule(7,8)
        #A class I made to contain the kinematics stuff, not all that neccesary
        self.kinematicsHandler=kinematicsHandlerClass()
    def setDrivetrainBasic(self,allSwerveSpeed,allSwerveRotation):
        #Passes a speed and rotation to all the swerve modules 
        #NOTE Cannot turn, only translational movement, not useable for actual competitons, TESTING ONLY
        for i in range(4):
            self.swerveModules[i].setState(allSwerveRotation,allSwerveSpeed)
            #cycles through each swerve and gives it commands
    def setDrivetrain(self,velocityMS:wpimath.geometry.Translation2d,radPS):
        swerveDesiredState=self.kinematicsHandler.calculateKinematics(velocityMS.x,velocityMS.y,radPS,self.compass.getRotation2d())
        #kinematics handler takes robot speeds relative to the field and turns them into swerve speeds
        for i in range(4):
            self.swerveModules[i].setState(swerveDesiredState[i].angle,swerveDesiredState[i].speed)
            #cycles through each swerve and gives it commands
class driveTrainCommand(commands2.Command):
    def __init__(self,driveSubsys:driveTrainSubsystem):
        self.addRequirements(driveSubsys)
        self.driveSubsys=driveSubsys
    def execute(self):
        self.driveSubsys.setDrivetrain(0,1)
class joystickSubsystem():
    #I dont put the commands2.Subsystem in here because its uneccesary since two or more commands can use this at once without breaking the robot
    def __init__(self,joystick:commands2.button.CommandGenericHID):
        self.joystick=joystick
    def getJoy(self):
        name=self.joystick._hid.getName()
        if name !=None:
            fb=self.joystick.getRawAxis(0)
            lr=self.joystick.getRawAxis(1)
            theta=self.joystick.getRawAxis(2)
            return fb,lr,theta