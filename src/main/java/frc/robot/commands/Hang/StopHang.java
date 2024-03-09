// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hang;

import frc.robot.subsystems.Hang;
import edu.wpi.first.wpilibj2.command.Command;

public class StopHang extends Command {
  /** Creates a new StopHangHooks. */
  private Hang hang;

  public StopHang(Hang hang) {
    this.hang = hang;
    addRequirements(hang);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hang.hangStop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hang.hangStop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}