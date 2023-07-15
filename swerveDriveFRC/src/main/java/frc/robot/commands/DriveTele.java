// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveTele extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drive m_drive;

  //is it gonna be rotation
  private double modifyInputs(double value, boolean isRot){
    if(isRot){
      //deadzone
      if(Math.abs(value) < 0.15){
        value = 0;
      
      }
      return value * Constants.DriveConstants.MAX_ANGULAR_VELOCITY;
    }

      else{
        if(Math.abs(value) < 0.15){
          value = 0;
        }
        return value * Constants.DriveConstants.MAX_TANGENTIAL_VELOCITY;
      }
  }

  public void driveFromChassis(ChassisSpeeds speeds){
    SwerveModuleState[] states = Constants.DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
    //gives it a cap, if you want speeds greater than you can ouput --> basically a clamp
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.DriveConstants.MAX_TANGENTIAL_VELOCITY);
    drive.setModuleStates(states);
  }

  //doubleSupplier = double
  private DoubleSupplier fwd, str, rot;
  private Drive drive;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveTele(DoubleSupplier fwd, DoubleSupplier str, DoubleSupplier rot, Drive drive) {
    this.m_drive = drive;
    this.fwd = fwd;
    this.str = str;
    this.rot = rot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double vx = modifyInputs(fwd.getAsDouble(), false);
    double vy = modifyInputs(str.getAsDouble(), false);
    double omega = modifyInputs(rot.getAsDouble(), true);

    //makes everything like a 3rd person robot
    //might have to add - in front 
    driveFromChassis(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, drive.getDriveHeading()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveFromChassis(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
