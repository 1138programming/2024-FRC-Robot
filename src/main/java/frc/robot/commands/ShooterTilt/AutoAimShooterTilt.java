// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterTilt;

import static frc.robot.Constants.ShooterTiltConstants.KShooterTiltCloseAimOffset;
import static frc.robot.Constants.ShooterTiltConstants.kShooterTiltDeadZone;
import static frc.robot.Constants.ShooterTiltConstants.kShooterTiltUpPos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

//import static frc.robot.Constants.TiltConstants.*;

import frc.robot.SubsystemUtil;
import frc.robot.subsystems.ShooterTilt;

public class AutoAimShooterTilt extends Command {
  /** Creates a new MoveShooterTiltTop. */
  private ShooterTilt shooterTilt;

  public AutoAimShooterTilt(ShooterTilt shooterTilt) {
    this.shooterTilt = shooterTilt;
    addRequirements(shooterTilt);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (SubsystemUtil.getIsNoteIndexed()) {
      shooterTilt.swivelToPosAbsolute(
          ShooterTilt.getAngleForShooterPivot(SubsystemUtil.getDistanceFromSpeaker()) + SmartDashboard.getNumber("Tilt Offset", KShooterTiltCloseAimOffset)
      );
    }
    else {
      shooterTilt.swivelToPosAbsolute(kShooterTiltUpPos);
    }
    SmartDashboard.putNumber("DISTANCE 2", SubsystemUtil.getDistanceFromSpeaker());
    SmartDashboard.putNumber("ANGLE TO SHOOT", ShooterTilt.getAngleForShooterPivot(SubsystemUtil.getDistanceFromSpeaker()) + 5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return (Math.abs(shooterTilt.getTiltEncoder() - kShooterTiltUpPos) < kShooterTiltDeadZone && !SubsystemUtil.getIsNoteIndexed());
  }
}
