// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ExampleConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Imports for CANSpark
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

public class DoSomethingExample extends SubsystemBase {
  private CANSparkMax exampleMotor;
  
  /** Creates a new DoSomethingExample. */
  public DoSomethingExample() {
    exampleMotor = new CANSparkMax(KExampleMotorID, MotorType.kBrushless);

    exampleMotor.setIdleMode(IdleMode.kCoast);
    exampleMotor.setInverted(KExampleMotorInverted);
  }
  
  public void spinExampleMotor(double speed) {
    exampleMotor.set(speed);
  }

  public void stopExampleMotor() {
    exampleMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
