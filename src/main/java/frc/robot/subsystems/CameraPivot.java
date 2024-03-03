// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.CameraConstants.KCameraPivotServoID;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraPivot extends SubsystemBase {
  private Servo servo;
  /** Creates a new CameraPivot. */
  public CameraPivot() {
    servo = new Servo(KCameraPivotServoID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
