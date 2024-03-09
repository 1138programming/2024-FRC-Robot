// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Flywheel;

import static frc.robot.Constants.FlywheelConstants.KFlywheelSpeed;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubsystemUtil;
import frc.robot.subsystems.Flywheel;
//import static frc.robot.Constants.FlywheelConstants.*;
import frc.robot.subsystems.ShooterTilt;

public class SpinFlywheelAutoAim extends Command {
  private Flywheel flywheel;
  private ShooterTilt shooterTilt;

  /** Creates a new SpinFlywheel. */
  public SpinFlywheelAutoAim(Flywheel flywheel, ShooterTilt shooterTilt) {
    this.flywheel = flywheel;
    this.shooterTilt = shooterTilt;
    
    addRequirements(flywheel, shooterTilt);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterTilt.swivelToPos(ShooterTilt.getMotorAngleFromShooterAngle(ShooterTilt.getAngleForShooterPivot(SubsystemUtil.getDistanceFromSpeaker())));
    SmartDashboard.putNumber("angle to shoot", ShooterTilt.getMotorAngleFromShooterAngle(ShooterTilt.getAngleForShooterPivot(SubsystemUtil.getDistanceFromSpeaker())));
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
