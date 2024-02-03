// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.*;

public class MoveHangHookDownSetPos extends Command {
  /** Creates a new CloseHang. */
  private Hang hang;
  private double hangSpeed;
  public MoveHangHookDownSetPos(Hang hang, double speed) {
    this.hang = hang;
    this.hangSpeed = hangSpeed;

    addRequirements(hang);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hang.setHangHookPosDown(hangSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hang.hangStop(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
