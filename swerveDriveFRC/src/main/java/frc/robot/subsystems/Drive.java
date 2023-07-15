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
import frc.robot.Util.SwerveModule;

import com.kauailabs.navx.frc.AHRS;
//import Util.swerveModule;

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
  }

  public static Drive getInstance(){
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void setModuleStates(SwerveModuleState[] states){
    frontLeft.setState(states[0]);
    frontRight.setState(states[1]);
    rearLeft.setState(states[2]);
    rearRight.setState(states[3]);
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition frontLeftPosition = new SwerveModulePosition(frontLeft.getDrivePosition(), frontLeft.getTurnAngle());
    SwerveModulePosition frontRightPosition = new SwerveModulePosition(frontRight.getDrivePosition(), frontRight.getTurnAngle());
    SwerveModulePosition rearLeftPosition = new SwerveModulePosition(rearLeft.getDrivePosition(), rearLeft.getTurnAngle());
    SwerveModulePosition rearRightPosition = new SwerveModulePosition(rearRight.getDrivePosition(), rearRight.getTurnAngle());

    SwerveModulePosition[] modulePositions = {frontLeftPosition, frontRightPosition, rearLeftPosition, rearRightPosition};
    return modulePositions;
  }

  public Rotation2d getDriveHeading(){
    return m_gyro.getRotation2d();
  }

  public void resetHeading(){
    m_gyro.reset();
  }

  public void logData(){
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
