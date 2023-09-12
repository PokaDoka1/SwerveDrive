// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Util.SwerveModule;

import com.kauailabs.navx.frc.AHRS;

public class Drive extends SubsystemBase {

  private static Drive instance = new Drive();

  private SwerveModule frontLeft = new SwerveModule(Constants.DriveConstants.FRONT_LEFT_DRIVE_PORT,Constants.DriveConstants.FRONT_LEFT_TURN_PORT,Constants.DriveConstants.FRONT_LEFT_ENCODER_PORT,Constants.DriveConstants.FRONT_LEFT_ENCODER_OFFSET, false);
  private SwerveModule frontRight = new SwerveModule(Constants.DriveConstants.FRONT_RIGHT_DRIVE_PORT,Constants.DriveConstants.FRONT_RIGHT_TURN_PORT,Constants.DriveConstants.FRONT_RIGHT_ENCODER_PORT,Constants.DriveConstants.FRONT_RIGHT_ENCODER_OFFSET, false);
  private SwerveModule rearLeft = new SwerveModule(Constants.DriveConstants.REAR_LEFT_DRIVE_PORT,Constants.DriveConstants.REAR_LEFT_TURN_PORT,Constants.DriveConstants.REAR_LEFT_ENCODER_PORT,Constants.DriveConstants.REAR_LEFT_ENCODER_OFFSET, false);
  private SwerveModule rearRight = new SwerveModule(Constants.DriveConstants.REAR_RIGHT_DRIVE_PORT,Constants.DriveConstants.REAR_RIGHT_TURN_PORT,Constants.DriveConstants.REAR_RIGHT_ENCODER_PORT,Constants.DriveConstants.REAR_RIGHT_ENCODER_OFFSET, false);

  public final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  /** Creates a new ExampleSubsystem. */
  public Drive() {
    m_gyro.reset();
    m_gyro.calibrate();
    
    SmartDashboard.putNumber("Turn P", 0);
    SmartDashboard.putNumber("Drive P", 0);
    SmartDashboard.putNumber("Drive FF", 0);
  }

  public static Drive getInstance(){
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logData();
  }

  public void setModuleStates(SwerveModuleState[] states){
    SmartDashboard.putNumber("front left desired velocity", states[1].speedMetersPerSecond);
    SmartDashboard.putNumber("front left desired angle", states[1].angle.getDegrees());

    frontLeft.setState(states[0]);
    frontRight.setState(states[1]);
    rearLeft.setState(states[2]);
    rearRight.setState(states[3]);
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] modulePositions = {frontLeft.getModulePosition(), frontRight.getModulePosition(), rearLeft.getModulePosition(), rearRight.getModulePosition()};
    return modulePositions;
  }

  public Rotation2d getDriveHeading(){
    return m_gyro.getRotation2d();
  }

  public void resetHeading(){
    m_gyro.reset();
  }

  public void logData(){
    SmartDashboard.putNumber("Drive Velocity", frontRight.getDriveVelocity());
    SmartDashboard.putNumber("Turn Angle", frontRight.getTurnAngle().getDegrees());

    SmartDashboard.putNumber("Absolute Turn", frontRight.getAbsoluteTurnAngle().getDegrees());

    SmartDashboard.putNumber("Gyro Degrees", m_gyro.getAngle());

    SmartDashboard.putNumber("front left abs", frontLeft.getAbsoluteTurnAngle().getDegrees());
    SmartDashboard.putNumber("front right abs", frontRight.getAbsoluteTurnAngle().getDegrees());
    SmartDashboard.putNumber("rear left abs", rearLeft.getAbsoluteTurnAngle().getDegrees());
    SmartDashboard.putNumber("rear right abs", rearRight.getAbsoluteTurnAngle().getDegrees());

    SmartDashboard.putNumber("front left", frontLeft.getTurnAngle().getDegrees());
    SmartDashboard.putNumber("front right", frontRight.getTurnAngle().getDegrees());
    SmartDashboard.putNumber("rear left", rearLeft.getTurnAngle().getDegrees());
    SmartDashboard.putNumber("rear right", rearRight.getTurnAngle().getDegrees());
    


  }

  public void readConfigGains(){
    frontLeft.configGains();
    frontRight.configGains();
    rearLeft.configGains();
    rearRight.configGains();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
