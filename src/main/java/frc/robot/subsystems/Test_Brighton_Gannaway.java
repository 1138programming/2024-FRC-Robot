// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Test_Brighton_Gannaway extends SubsystemBase {
  /** Creates a new Test_Brighton_Gannaway. */
  private CANSparkMax canSparkMax; //motor
  private DigitalInput digitalInput;

  public Test_Brighton_Gannaway() {
     canSparkMax = new CANSparkMax(123456, MotorType.kBrushless);
     digitalInput = new DigitalInput(0);
  }

  public void move_CANSparkMax(double spd) {
    canSparkMax.set(spd);
  }

  public void stop_CANSparkMax () {
    canSparkMax.stopMotor();
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }

}
