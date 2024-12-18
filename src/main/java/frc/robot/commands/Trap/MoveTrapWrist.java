// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Trap;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Trap;

public class MoveTrapWrist extends Command {
  /** Creates a new MoveTrapWrist. */
  private Trap trap;
  private double speed;
  public MoveTrapWrist(Trap trap, double speed) {
    this.trap = trap;
    this.speed = speed; 
    addRequirements(trap);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    trap.moveWristMotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
