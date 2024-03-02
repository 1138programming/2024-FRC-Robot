// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Flywheel;

import static frc.robot.Constants.FlywheelConstants.KFlywheelSpeed;
import static frc.robot.Constants.FlywheelConstants.KFlywheelVelocity;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;
//import static frc.robot.Constants.FlywheelConstants.*;

public class SpinUpFlywheel extends Command {
  private Flywheel flywheel;

  /** Creates a new SpinFlywheel. */
  public SpinUpFlywheel(Flywheel flywheel) {
    this.flywheel = flywheel;
    addRequirements(flywheel);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    flywheel.spinFlywheel(KFlywheelSpeed);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(flywheel.getUpperMotorVelocity() - KFlywheelVelocity) <= 300;
  }
}
