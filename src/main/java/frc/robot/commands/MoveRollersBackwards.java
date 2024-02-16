// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Trap;
import frc.robot.Constants;

public class MoveRollersBackwards extends Command {
  /** Creates a new MoveRollersBackwards. */
  private Trap trap;
  public MoveRollersBackwards(Trap trap) {
    this.trap = trap;
    addRequirements(trap);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    trap.moveTrapRollers(-0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    trap.stopRollers();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !trap.getTrapNoteSensor();
  }
}
