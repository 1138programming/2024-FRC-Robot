// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Flywheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubsystemUtil;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.ShooterTilt;


import static frc.robot.Constants.FlywheelConstants.KFlywheelCloseSpeed;
import static frc.robot.Constants.FlywheelConstants.KFlywheelCloseSpeedMaxDistance;
import static frc.robot.Constants.FlywheelConstants.KFlywheelFarSpeed;
import static frc.robot.Constants.ShooterTiltConstants.KShooterTiltCloseAimOffset;
import static frc.robot.Constants.ShooterTiltConstants.KShooterTiltMediumAimOffset;
import static frc.robot.Constants.ShooterTiltConstants.KShooterTiltAuton1Angle;
import static frc.robot.Constants.SwerveDriveConstants.KRotationD;
import static frc.robot.Constants.SwerveDriveConstants.KRotationI;
import static frc.robot.Constants.SwerveDriveConstants.KRotationP;
import static frc.robot.Constants.*;

public class SpinFlywheelAndTiltShuttle extends Command {
  /** Creates a new ShooterShuttleShoot. */
  private Flywheel flywheel;
  private ShooterTilt shooterTilt;

  private PIDController rotController;
  public SpinFlywheelAndTiltShuttle(Flywheel flywheel, ShooterTilt shooterTilt) {
    this.flywheel = flywheel;
    this.shooterTilt = shooterTilt;
    rotController = new PIDController(KRotationP, KRotationI, KRotationD);
    rotController.enableContinuousInput(-180, 180);
    
    addRequirements(flywheel, shooterTilt);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterTilt.swivelToPosAbsolute(KShooterTiltAuton1Angle);

    flywheel.spinFlywheel(KFlywheelFarSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
