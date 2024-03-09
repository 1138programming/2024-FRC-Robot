// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Trap;

import static frc.robot.Constants.TrapConstants.KTrapWristAmp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Trap;

public class WristToPos extends Command {
  /** Creates a new MoveWristUp. */
  private Trap trap;
  private double pos;
  
  public WristToPos(Trap trap, double pos) {
    this.trap = trap;
    this.pos = pos;
    addRequirements(trap);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    trap.swivelToPos(pos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    trap.stopWrist();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return trap.getPotentiometer() >= KTrapWristAmp;
  }
}
