// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static frc.robot.Constants.Hang.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;

// Neo Motor not sure yet for sparkmax
// They said that we need 2 piston pneumatics
// an encoder and 2 limit switches but might be changed 

public class Hang extends SubsystemBase {
  /** Creates a new Hang. */
  private CANSparkMax hangMotor;

  public Hang() {
    hangMotor = new CANSparkMax(KHangMotorID, MotorType.kBrushless);//ID isn't working
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
