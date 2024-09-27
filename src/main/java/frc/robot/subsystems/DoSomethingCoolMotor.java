// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeConstants.KIntakeMotorID;
import static frc.robot.Constants.IntakeConstants.KIntakeMotorIsInverted;
import static frc.robot.Constants.IntakeConstants.exampleMotorInverted;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

public class DoSomethingCoolMotor extends SubsystemBase {
  private CANSparkMax exampleMotor;
  /** Creates a new DoSomethingCoolMotor. */
  public DoSomethingCoolMotor() {
    exampleMotor = new CANSparkMax(KIntakeMotorID, MotorType.kBrushless);
    exampleMotor.setIdleMode(IdleMode.kCoast);
    exampleMotor.setInverted(exampleMotorInverted);
  }
  public void spinExampleMotor(double speed){
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
