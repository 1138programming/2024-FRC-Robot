// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;
//Constants Import
import static frc.robot.Constants.FlywheelConstants.*;


public class SpinLowerFlywheel extends Command {
  private Flywheel flywheel;

  /** Creates a new SpinLowerFlywheel. */
  public SpinLowerFlywheel(Flywheel flywheel) {
    addRequirements(flywheel);
    this.flywheel = flywheel;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    flywheel.spinLowerFlywheel(KFlywheelSpeedLower);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.stopMotors();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
