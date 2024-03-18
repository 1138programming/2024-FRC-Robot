// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Flywheel;

import static frc.robot.Constants.FlywheelConstants.KFlywheelSpeed;
import static frc.robot.Constants.ShooterTiltConstants.KShooterTiltAimOffset;
import static frc.robot.Constants.SwerveDriveConstants.KBaseRotMaxPercent;
import static frc.robot.Constants.SwerveDriveConstants.KPhysicalMaxDriveSpeedMPS;
import static frc.robot.Constants.SwerveDriveConstants.KRotationD;
import static frc.robot.Constants.SwerveDriveConstants.KRotationI;
import static frc.robot.Constants.SwerveDriveConstants.KRotationP;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubsystemUtil;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Flywheel;
//import static frc.robot.Constants.FlywheelConstants.*;
import frc.robot.subsystems.ShooterTilt;

public class SpinFlywheelAndRotate extends Command {
  private Flywheel flywheel;
  private Base base;
  private ShooterTilt shooterTilt;

  private PIDController rotController;

  /** Creates a new SpinFlywheel. */
  public SpinFlywheelAndRotate(Flywheel flywheel, Base base, ShooterTilt shooterTilt) {
    this.flywheel = flywheel;
    this.base = base;
    this.shooterTilt = shooterTilt;
    rotController = new PIDController(KRotationP, KRotationI, KRotationD);
    rotController.enableContinuousInput(-180, 180);
    
    addRequirements(flywheel, base, shooterTilt);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotController.setP(SmartDashboard.getNumber("RotP", KRotationP));
    rotController.setI(SmartDashboard.getNumber("RotI", KRotationI));
    rotController.setD(SmartDashboard.getNumber("RotD", KRotationD));

    double rotSpeed = rotController.calculate(base.getAimingHeadingDeg(), base.getAngleFromSpeaker());
    // double rotSpeed = rotController.calculate(base.getAprilTagOffsetFromSpeaker(), 0);

    base.drive(0, 0, rotSpeed, true, KPhysicalMaxDriveSpeedMPS, KBaseRotMaxPercent * base.getRotSpeedFactor());

    SmartDashboard.putNumber("base.getAngleFromSpeaker()",  base.getAngleFromSpeaker());
    SmartDashboard.putNumber("base.getPositiveHeadingDeg()",  base.getPositiveHeadingDeg());
    SmartDashboard.putNumber("base.getAprilTagOffsetFromSpeaker()", base.getAprilTagOffsetFromSpeaker());
    
    shooterTilt.swivelToPosAbsolute(
      ShooterTilt.getAngleForShooterPivot(SubsystemUtil.getDistanceFromSpeaker()) + KShooterTiltAimOffset
    );

    flywheel.spinFlywheel(KFlywheelSpeed);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.stopMotors();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
