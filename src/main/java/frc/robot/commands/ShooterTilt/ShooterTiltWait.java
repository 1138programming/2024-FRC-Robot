// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterTilt;

import frc.robot.SubsystemUtil;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.ShooterTilt;

import static frc.robot.Constants.ShooterTiltConstants.kShooterTiltUpPos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterTiltWait extends Command {
  /** Creates a new ShooterTiltStop. */
  private ShooterTilt shooterTilt;
  private LEDs leds;

  public ShooterTiltWait(ShooterTilt shooterTilt, LEDs leds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterTilt = shooterTilt;
    this.leds = leds;
    addRequirements(shooterTilt, leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leds.setLEDStripRed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isNoteIndexed = SubsystemUtil.getIsNoteIndexed();
    SmartDashboard.putBoolean("isNoteIndexed", isNoteIndexed);
    if (!isNoteIndexed) {
      shooterTilt.swivelToPosAbsolute(kShooterTiltUpPos);
    }
    else {
      shooterTilt.swivelToPosAbsolute(kShooterTiltUpPos);
    }
    // boolean isNoteIndexed = SubsystemUtil.getIsNoteIndexed();
    // SmartDashboard.putBoolean("isNoteIndexed", isNoteIndexed);
    // if (!isNoteIndexed) {
    //   shooterTilt.swivelToPos(ShooterTilt.getMotorAngleFromShooterAngle(kShooterTiltUpPos));
    // }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leds.setLEDStripGreen();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
