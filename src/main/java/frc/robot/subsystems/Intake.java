// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.IntakeConstants.KIntakeMotorID;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import for CANSparkMax
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

import com.ctre.phoenix6.hardware.TalonFX;
public class Intake extends SubsystemBase {
  private CANSparkMax intakeMotor;
  /** Creates a new Intake. */

  public Intake() {
    intakeMotor = new CANSparkMax(KIntakeMotorID, MotorType.kBrushless);
  }
  
  public void spinIntake(double speed){
    intakeMotor.set(speed);
  }

  public void intakeSpinStop(){
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
