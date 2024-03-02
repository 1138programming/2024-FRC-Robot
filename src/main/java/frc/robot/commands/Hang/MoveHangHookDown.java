// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hang;

import frc.robot.subsystems.Hang;

import static frc.robot.Constants.HangConstants.KHangMotorSpeedDown;

import edu.wpi.first.wpilibj2.command.Command;

public class MoveHangHookDown extends Command {
  /** Creates a new MoveHangDown. */
  private Hang hang;

  public MoveHangHookDown(Hang hang) {
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
