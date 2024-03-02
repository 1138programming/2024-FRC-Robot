// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import static frc.robot.Constants.LimelightConstants.KLimelightAngleDeadzone;
import static frc.robot.Constants.LimelightConstants.KlimelightrotControl;
import static frc.robot.Constants.LimelightConstants.KspeakerCoordinatesBlue;
import static frc.robot.Constants.LimelightConstants.KspeakerCoordinatesRed;
import static frc.robot.Constants.SwerveDriveConstants.KBaseRotMaxPercent;
import static frc.robot.Constants.SwerveDriveConstants.KMaxAngularSpeed;
import static frc.robot.Constants.SwerveDriveConstants.KPhysicalMaxDriveSpeedMPS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Base;

public class RotateToSpeaker extends Command {
  private Base base;
  private double rotSpeed;
  private double offset;  

  /** Creates a new RotateToSpeaker. */
  public RotateToSpeaker(Base base) {
    this.base = base;
    addRequirements(base);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (DriverStation.getAlliance().toString() == "blue") {
      offset = Math.atan(Math.abs(KspeakerCoordinatesBlue[1]-base.getRobotPoseY()) / Math.abs(KspeakerCoordinatesBlue[0]-base.getRobotPoseX()));
      rotSpeed = KlimelightrotControl.calculate(base.getHeadingDeg(), offset);
    } 
    else {
      offset = Math.atan(Math.abs(KspeakerCoordinatesRed[1]-base.getRobotPoseY()) / Math.abs(KspeakerCoordinatesRed[0]-base.getRobotPoseX()));
      rotSpeed = KlimelightrotControl.calculate(base.getHeadingDeg(), offset);
    }
    base.drive(0, 0, rotSpeed, true, KPhysicalMaxDriveSpeedMPS, KMaxAngularSpeed * KBaseRotMaxPercent);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (KLimelightAngleDeadzone >= Math.abs(base.getHeadingDeg() - offset)); 
  }
}