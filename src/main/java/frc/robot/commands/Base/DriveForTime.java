// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import static frc.robot.Constants.SwerveDriveConstants.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Base;

public class DriveForTime extends Command {
 private final Base base;
 private final double speed;
 private final double time;
 private double initialTime;

//  private double kRotationP = 0.005;
//   private double kRotationI = 0;
//   private double kRotationD = 0;

//   private PIDController rotationCorrectionPID;

  /** Creates a new DriveWithJoySticks. */
  public DriveForTime(Base base, double speed, double time) {
    this.base = base;
    this.speed = speed;
    this.time = time;
    
    addRequirements(base);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialTime = Timer.getFPGATimestamp();
    base.resetAllRelEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    base.drive(speed, 0, 0, false, KPhysicalMaxDriveSpeedMPS * base.getDriveSpeedFactor(), KBaseRotMaxPercent * base.getRotSpeedFactor());
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return initialTime + time <= Timer.getFPGATimestamp();
  }
}
