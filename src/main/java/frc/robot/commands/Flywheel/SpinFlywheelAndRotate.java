// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Flywheel;

import static frc.robot.Constants.FlywheelConstants.KFlywheelCloseSpeed;
import static frc.robot.Constants.FlywheelConstants.KFlywheelCloseSpeedMaxDistance;
import static frc.robot.Constants.FlywheelConstants.KFlywheelFarSpeed;
import static frc.robot.Constants.FlywheelConstants.KFlywheelTiltUpDistance;
import static frc.robot.Constants.ShooterTiltConstants.KShooterTiltCloseAimOffset;
import static frc.robot.Constants.ShooterTiltConstants.KShooterTiltFarAimOffset;
import static frc.robot.Constants.ShooterTiltConstants.KShooterTiltMediumAimOffset;
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
  private double rotSpeed;

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
  public void initialize() {
    flywheel.setCoastMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotController.setP(SmartDashboard.getNumber("RotP", KRotationP));
    rotController.setI(SmartDashboard.getNumber("RotI", KRotationI));
    rotController.setD(SmartDashboard.getNumber("RotD", KRotationD));

    rotSpeed = rotController.calculate(base.getAngleFromSpeaker(), 0);
    // double rotSpeed = rotController.calculate(base.getAimingHeadingDeg(),
    // base.getAngleFromSpeaker());
    // double rotSpeed =
    // rotController.calculate(base.getAprilTagOffsetFromSpeaker(), 0);

    base.drive(0, 0, -rotSpeed, true, KPhysicalMaxDriveSpeedMPS * base.getDriveSpeedFactor(), KBaseRotMaxPercent * base.getRotSpeedFactor());

    if (SubsystemUtil.getDistanceFromSpeaker() < KFlywheelCloseSpeedMaxDistance) {
      shooterTilt.swivelToPosAbsolute(
          ShooterTilt.getAngleForShooterPivot(SubsystemUtil.getDistanceFromSpeaker())
              + SmartDashboard.getNumber("Tilt Offset Close", KShooterTiltCloseAimOffset));
      flywheel.spinFlywheel(KFlywheelCloseSpeed);
    } else if (SubsystemUtil.getDistanceFromSpeaker() < 6) {
      shooterTilt.swivelToPosAbsolute(
          ShooterTilt.getAngleForShooterPivot(SubsystemUtil.getDistanceFromSpeaker())
              + SmartDashboard.getNumber("Tilt Offset Medium", KShooterTiltMediumAimOffset));
      flywheel.spinFlywheel(KFlywheelFarSpeed);
    } else {
      shooterTilt.swivelToPosAbsolute(
          ShooterTilt.getAngleForShooterPivot(SubsystemUtil.getDistanceFromSpeaker())
              + SmartDashboard.getNumber("Tilt Offset Far", KShooterTiltFarAimOffset));
      flywheel.spinFlywheel(KFlywheelFarSpeed);
    }

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
