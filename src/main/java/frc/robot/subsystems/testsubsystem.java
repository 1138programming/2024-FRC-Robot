// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class testsubsystem extends SubsystemBase {
  /** Creates a new testsubsystem. */
  private CANSparkMax spark;
  private DigitalInput xyz;
  public testsubsystem() {
    spark = new CANSparkMax(1,MotorType.kBrushless);
  xyz = new DigitalInput(0);
  
  }
public void movemotor(double speed){
  spark.set(speed);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
