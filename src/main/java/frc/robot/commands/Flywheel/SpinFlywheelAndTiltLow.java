// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Flywheel;

import static frc.robot.Constants.FlywheelConstants.KFlywheelCloseSpeed;
import static frc.robot.Constants.FlywheelConstants.KFlywheelCloseSpeedMaxDistance;
import static frc.robot.Constants.FlywheelConstants.KFlywheelFarSpeed;
import static frc.robot.Constants.ShooterTiltConstants.KShooterTiltCloseAimOffset;
import static frc.robot.Constants.SwerveDriveConstants.KRotationD;
import static frc.robot.Constants.SwerveDriveConstants.KRotationI;
import static frc.robot.Constants.SwerveDriveConstants.KRotationP;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubsystemUtil;
import frc.robot.subsystems.Flywheel;
//import static frc.robot.Constants.FlywheelConstants.*;
import frc.robot.subsystems.ShooterTilt;

public class SpinFlywheelAndTiltLow extends Command {
  private Flywheel flywheel;
  private ShooterTilt shooterTilt;

  private PIDController rotController;

  /** Creates a new SpinFlywheel. */
  public SpinFlywheelAndTiltLow(Flywheel flywheel, ShooterTilt shooterTilt) {
    this.flywheel = flywheel;
    this.shooterTilt = shooterTilt;
    rotController = new PIDController(KRotationP, KRotationI, KRotationD);
    rotController.enableContinuousInput(-180, 180);
    
    addRequirements(flywheel, shooterTilt);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterTilt.swivelToPosAbsolute(
      ShooterTilt.getAngleForShooterPivot(SubsystemUtil.getDistanceFromSpeaker()) + KShooterTiltCloseAimOffset);

    if (SubsystemUtil.getDistanceFromSpeaker() < KFlywheelCloseSpeedMaxDistance) {
      flywheel.spinFlywheel(KFlywheelCloseSpeed);
    } else {
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
