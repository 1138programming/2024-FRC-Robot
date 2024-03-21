// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import static frc.robot.Constants.SwerveDriveConstants.KBaseRotMaxPercent;
import static frc.robot.Constants.SwerveDriveConstants.KPhysicalMaxDriveSpeedMPS;
import static frc.robot.Constants.SwerveDriveConstants.KRotationD;
import static frc.robot.Constants.SwerveDriveConstants.KRotationI;
import static frc.robot.Constants.SwerveDriveConstants.KRotationP;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Hang;

public class AmpDrivingMode extends Command {
  private Base base;
  private Hang hang;

  private double fbSpeed; //Speed of the robot in the x direction (forward).
  private double lrSpeed; //Speed of the robot in the Y direction (sideways).
  private double rot;

  private PIDController rotController;

  private double angle;

  /** Creates a new AmpDrivingMode. */
  public AmpDrivingMode(Base base, Hang hang) {
    this.base = base;
    this.hang = hang;

    rotController = new PIDController(KRotationP, KRotationI, KRotationD);
    rotController.enableContinuousInput(-180, 180);

    angle = -90;

    addRequirements(base, hang);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hang.moveHangPistonsUp();
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if(alliance.get() == DriverStation.Alliance.Blue) {
        angle = -90;
      }
      if(alliance.get() == DriverStation.Alliance.Red) {
        angle = 90;
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    fbSpeed = Robot.m_robotContainer.getLogiLeftYAxis();
    lrSpeed = Robot.m_robotContainer.getLogiLeftXAxis();
    rot = rotController.calculate(angle, base.getAngleFromSpeaker());

    base.drive(fbSpeed, lrSpeed, rot, true, KPhysicalMaxDriveSpeedMPS * base.getDriveSpeedFactor(), KBaseRotMaxPercent * base.getRotSpeedFactor());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hang.moveHangPistonsDown();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
