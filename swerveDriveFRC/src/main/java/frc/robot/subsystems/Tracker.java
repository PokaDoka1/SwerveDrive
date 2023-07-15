// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Tracker extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.DriveConstants.DRIVE_KINEMATICS, Drive.getInstance().getDriveHeading(), Drive.getInstance().getModulePositions());
  
  private static Tracker instance = new Tracker();

  private final Field2d field = new Field2d();

  private Drive drive = Drive.getInstance();

  public Tracker() {
    SmartDashboard.putData("Field", field);
  }

  public void updatePose(){
    odometry.update(drive.getDriveHeading(), drive.getModulePositions());
    field.setRobotPose(odometry.getPoseMeters());
  }

  public void setOdometry(Pose2d pose){
    odometry.resetPosition(drive.getDriveHeading(), drive.getModulePositions(), pose);
  }

  public Pose2d getOdometry(){
    return odometry.getPoseMeters();
  }

  public void resetHeading(){
    odometry.resetPosition(drive.getDriveHeading(), drive.getModulePositions(), new Pose2d(getOdometry().getTranslation(), Rotation2d.fromDegrees(0)));
  }

  public void logData(){
    SmartDashboard.putNumber("pose x", getOdometry().getX());
    SmartDashboard.putNumber("pose y", getOdometry().getY());
    SmartDashboard.putNumber("post rot", getOdometry().getRotation().getDegrees());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePose();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
