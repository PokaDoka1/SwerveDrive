package frc.robot.Util;



import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.SwerveModulesConstants;


public class SwerveModule{
    CANSparkMax m_driveMotor;
    CANSparkMax m_turnMotor;

    SparkMaxPIDController m_driverController;
    SparkMaxPIDController m_turnController;

    RelativeEncoder m_turnEncoder;
    RelativeEncoder m_driveEncoder;

    private final CANCoder m_canCoder;


    public SwerveModule(int drivePort, int turnPort, int encoderPort, double angleOffset, boolean isInverted){
        //angleOffset = offset of cancoders

        m_driveMotor = new CANSparkMax(drivePort, MotorType.kBrushless);
        m_turnMotor = new CANSparkMax(turnPort, MotorType.kBrushless);
        m_canCoder = new CANCoder(encoderPort);

        //create PID controllers
        m_driverController = m_driveMotor.getPIDController();
        m_turnController = m_turnMotor.getPIDController();

        m_driveEncoder = m_driveMotor.getEncoder();
        m_turnEncoder = m_turnMotor.getEncoder();

        //refreshes your motors and encoder
        m_driveMotor.restoreFactoryDefaults();
        m_turnMotor.restoreFactoryDefaults();
        m_canCoder.configFactoryDefault();

        m_driveMotor.setIdleMode(IdleMode.kBrake);
        m_driveMotor.setInverted(isInverted);
        //first argument (stall limit) refers to how much amperage a motor can pull while trying to overcome resistance
        //second argument (free limit) refers to how much amperage a motor can pull with no resistance.
        m_driveMotor.setSmartCurrentLimit(15, 15);

        m_driverController.setP(SwerveModulesConstants.DRIVE_KP);
        //feedforward = static voltage 
        //voltage you add to give it a little boost
        m_driverController.setFF(SwerveModulesConstants.DRIVE_KF);

        m_turnMotor.setIdleMode(IdleMode.kBrake);
        m_turnMotor.setInverted(false);
        m_turnMotor.setSmartCurrentLimit(15,15);
        m_turnController.setP(SwerveModulesConstants.TURN_KP);

        //m_driveEncoder.setInverted(false);
        //m_turnEncoder.setInverted(false);
        
        m_driveEncoder.setPosition(0);
        m_turnEncoder.setPosition(0);

        //sensor range is -180 to 180
        m_canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        //if cancoder stand still, it will reset back to 0, but now it will keep it's value
        //????
        m_canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        m_canCoder.configMagnetOffset(angleOffset);

        //rotations * gear ratio
        //12.8 motor rotations 

        //getAbsolutePosition = degrees
        //degrees = 360 = fraction 
        //turnGearRatio = number of turns the motor must go through to turn wheel one rotation.
        //m_turnEncoder.setPosition((m_canCoder.getAbsolutePosition() / (360))  * (SwerveModulesConstants.TURN_GEAR_RATIO));
    }

    public void setTurnDegrees(Rotation2d turnSetpoint){
        //set the controller --> getDegrees() 

        //wholeModule rotation * gear ratio 
        //conversion from degrees to steps --> rotations
        m_turnController.setReference((turnSetpoint.getRotations()) * SwerveModulesConstants.TURN_GEAR_RATIO, ControlType.kPosition);
    }

    public void setDriveVelocity(double metersPerSec){
        //rotations per minute

        if(metersPerSec == 0){
            m_driveMotor.set(0);
        }
        
        m_driverController.setReference(

            //metres divided by circumference 
            //meters = rotations * circumference 
            //meters --> rotation

            //gear ratio * rotations per minute
            Constants.SwerveModulesConstants.DRIVE_GEAR_RATIO * ((metersPerSec * 60) / (Math.PI * Constants.SwerveModulesConstants.WHEEL_DIAMETER_METERS)),
            ControlType.kVelocity);
    }

    public void setState(SwerveModuleState state){
        //optimizing = gets path of least resistance 
        //state = velocity and turn setpoint
        SwerveModuleState optimizedState = CTREUtils.optimize(state, getTurnAngle());
        setDriveVelocity(optimizedState.speedMetersPerSecond);
        setTurnDegrees(optimizedState.angle);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), getTurnAngle());
    }

    public SwerveModulePosition getModulePosition(){
        return new SwerveModulePosition(getDrivePosition(), getTurnAngle());
    }

    public Rotation2d getTurnAngle(){
        return Rotation2d.fromRotations(m_turnEncoder.getPosition() / Constants.SwerveModulesConstants.TURN_GEAR_RATIO);
    }

    public Rotation2d getAbsoluteTurnAngle(){
        return Rotation2d.fromDegrees(m_canCoder.getAbsolutePosition());
    }

    public double getDriveVelocity(){
        //convert velocity (RPM) --> rotations per second --> multiply by circumference
        //convert to like metersPerSecond rather than RPM
        return (m_driveEncoder.getVelocity() / (Constants.SwerveModulesConstants.DRIVE_GEAR_RATIO * 60)) * (Math.PI * Constants.SwerveModulesConstants.WHEEL_DIAMETER_METERS);
    }

    public double getDrivePosition(){
        //meters 
        //rotations / gear ratio * circumference
        return (m_driveEncoder.getPosition() / (Constants.SwerveModulesConstants.DRIVE_GEAR_RATIO)) * Math.PI * Constants.SwerveModulesConstants.WHEEL_DIAMETER_METERS;
    }

    public void rezeroTurnMotor(){
       m_turnEncoder.setPosition(m_canCoder.getAbsolutePosition() / (360)  * (SwerveModulesConstants.TURN_GEAR_RATIO));
    }

    public void configGains(){
        m_turnController.setP(SmartDashboard.getNumber("Turn P", 0));
        m_driverController.setP(SmartDashboard.getNumber("Drive P", 0));

        m_driverController.setFF(SmartDashboard.getNumber("Drive FF", 0));
    }
}