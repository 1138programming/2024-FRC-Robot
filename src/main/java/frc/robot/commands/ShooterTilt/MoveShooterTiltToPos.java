// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterTilt;

import static frc.robot.Constants.ShooterTiltConstants.kShooterTiltDeadZone;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterTilt;

public class MoveShooterTiltToPos extends Command {
  /** Creates a new MoveShooterTiltToPos. */    
  //private MoveShooterTiltToPos moveShooterTiltToPos;
  private ShooterTilt shooterTilt;
  private double pos;

  public MoveShooterTiltToPos(ShooterTilt shooterTilt, double pos) {
    this.shooterTilt = shooterTilt;
    this.pos = pos;
    addRequirements(shooterTilt);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterTilt.swivelToPos(ShooterTilt.getMotorAngleFromShooterAngle(pos));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    SmartDashboard.putNumber("isFinished num", Math.abs(shooterTilt.getTiltEncoder() - ShooterTilt.getMotorAngleFromShooterAngle(SmartDashboard.getNumber("Swivel To", 18))));
    return (Math.abs(shooterTilt.getTiltEncoder() - ShooterTilt.getMotorAngleFromShooterAngle(SmartDashboard.getNumber("Swivel To", 18)) ) < kShooterTiltDeadZone);
  }
}
