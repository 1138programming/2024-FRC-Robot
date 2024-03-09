// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import static frc.robot.Constants.LimelightConstants.KLimelightAngleDeadzone;
import static frc.robot.Constants.SwerveDriveConstants.KBaseRotMaxPercent;
import static frc.robot.Constants.SwerveDriveConstants.KMaxAngularSpeed;
import static frc.robot.Constants.SwerveDriveConstants.KPhysicalMaxDriveSpeedMPS;
import static frc.robot.Constants.SwerveDriveConstants.KRotationD;
import static frc.robot.Constants.SwerveDriveConstants.KRotationI;
import static frc.robot.Constants.SwerveDriveConstants.KRotationP;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Base;

public class RotateToAngle extends Command {
 private final Base base;
 private PIDController rotController;
 private double angle;

  private double rot;

  /** Creates a new DriveWithJoySticks. */
  public RotateToAngle(Base base, double angle) {
    this.base = base;
    this.angle = angle;

    rotController = new PIDController(
      SmartDashboard.getNumber("RotP", KRotationP), 
      SmartDashboard.getNumber("RotI", KRotationI), 
      SmartDashboard.getNumber("RotD", KRotationD)
    );

    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    base.resetAllRelEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  
    rot = rotController.calculate(base.getHeadingDeg(), angle);
    base.drive(0, 0, rot, true, KPhysicalMaxDriveSpeedMPS * base.getDriveSpeedFactor(), KMaxAngularSpeed * KBaseRotMaxPercent);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (KLimelightAngleDeadzone >= Math.abs(base.getHeadingDeg() - base.getAngleFromSpeaker()));
  }
}
