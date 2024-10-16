// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterTilt;

import static frc.robot.Constants.ShooterTiltConstants.*;

import edu.wpi.first.wpilibj2.command.Command;

//import static frc.robot.Constants.TiltConstants.*;

import frc.robot.subsystems.ShooterTilt;

public class MoveShooterTiltTop extends Command {
  /** Creates a new MoveShooterTiltTop. */
  private ShooterTilt shooterTilt;
  public MoveShooterTiltTop(ShooterTilt shooterTilt) {
    this.shooterTilt = shooterTilt;
    addRequirements(shooterTilt);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterTilt.swivelToPos(ShooterTilt.getMotorAngleFromShooterAngle(kShooterTiltUpPos));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(shooterTilt.getTiltEncoder() - kShooterTiltUpPos) < kShooterTiltDeadZone);
  }
}
