// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import static frc.robot.Constants.LimelightConstants.*;
import static frc.robot.Constants.SwerveDriveConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Limelight;

public class CenterBaseOnAprilTag extends Command {
  private Base base;
  private Limelight limelight;
  private double xOffsetFromAprilTag;

  /** Creates a new CenterBaseOnAprilTag. */
  public CenterBaseOnAprilTag(Base base, Limelight limelight) {

    this.base = base;
    this.limelight = limelight;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(base, limelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int id = (int) limelight.getTID();
    if (DriverStation.getAlliance().toString() == "blue") {
      if (limelight.getTargetFound() && id == KspeakerAprilTagsBlue[1]) {
        xOffsetFromAprilTag = limelight.getXAngle();
      } 
      else if (limelight.getTargetFound() && id == KspeakerAprilTagsBlue[0]) {
        xOffsetFromAprilTag = limelight.getXAngle() - KaprilTagOffset;
      }
    } 
    else {
      if (limelight.getTargetFound() && id == KspeakerAprilTagsBlue[1]) {
        xOffsetFromAprilTag = limelight.getXAngle();
      } 
      else if (limelight.getTargetFound() && id == KspeakerAprilTagsBlue[1]) {
        xOffsetFromAprilTag = limelight.getXAngle() - KaprilTagOffset;
      }
    }

    double rotSpeed = KlimelightrotControl.calculate(xOffsetFromAprilTag, 0);

    base.drive(0, 0, rotSpeed, false, KPhysicalMaxDriveSpeedMPS, KMaxAngularSpeed * KBaseRotMaxPercent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (1 > Math.abs(xOffsetFromAprilTag - KLimelightAngleDeadzone)) {
      return true;
    } else {
      return false;
    }
  }
}
