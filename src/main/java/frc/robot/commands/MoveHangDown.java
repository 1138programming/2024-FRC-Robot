// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Subsystems.*;
import frc.robot.Constants;

import static frc.robot.Constants.Hang.KHangMotorSpeedDown;

import edu.wpi.first.wpilibj2.command.Command;

public class MoveHangDown extends Command {
  /** Creates a new MoveHangDown. */
  private Hang hang;

  public MoveHangDown(Hang hang) {
    this.hang = hang;
    addRequirements(hang);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hang.moveHangMotor(KHangMotorSpeedDown);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hang.hangStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
