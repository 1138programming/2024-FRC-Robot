// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.TiltConstants.KTiltMotorSpeed;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Tilt;

public class TiltSpinDown extends Command {
  /** Creates a new TiltSpinDown. */
  private Tilt tilt;

  public TiltSpinDown(Tilt tilt) {
    this.tilt = tilt;
    addRequirements(tilt);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tilt.spinTiltMotorD(KTiltMotorSpeed);
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
