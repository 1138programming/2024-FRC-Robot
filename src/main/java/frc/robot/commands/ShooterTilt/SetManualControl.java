// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterTilt;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterTilt;

public class SetManualControl extends Command {
  private ShooterTilt shooterTilt;
  /** Creates a new SetManualControl. */
  public SetManualControl(ShooterTilt shooterTilt) {
    this.shooterTilt = shooterTilt;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterTilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterTilt.toggleManualControl();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
