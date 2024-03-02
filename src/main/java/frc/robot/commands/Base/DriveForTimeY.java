// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import static frc.robot.Constants.SwerveDriveConstants.KBaseRotMaxPercent;
import static frc.robot.Constants.SwerveDriveConstants.KPhysicalMaxDriveSpeedMPS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Base;

public class DriveForTimeY extends Command {
 private final Base base;
 private  double speed;
 private double initialSpeed;
 private final double time;
 private double initialTime;

//  private double kRotationP = 0.005;
//   private double kRotationI = 0;
//   private double kRotationD = 0;

//   private PIDController rotationCorrectionPID;

  /** Creates a new DriveWithJoySticks. */
  public DriveForTimeY(Base base, double initialSpeed, double time) {
    this.base = base;
    this.initialSpeed = initialSpeed;
    this.time = time;
    
    addRequirements(base);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialTime = Timer.getFPGATimestamp();
    base.resetAllRelEncoders();
    if (DriverStation.getAlliance().toString().equals("Optional[Blue]") ) {
      speed = -initialSpeed;
    }
    else {
      speed = initialSpeed;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    base.drive(0, speed, 0, true, KPhysicalMaxDriveSpeedMPS * base.getDriveSpeedFactor(), KBaseRotMaxPercent * base.getRotSpeedFactor());
    SmartDashboard.putNumber("Timer", Timer.getFPGATimestamp());
    SmartDashboard.putNumber("whole time", initialTime + time);
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
