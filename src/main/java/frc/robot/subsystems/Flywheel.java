// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix.sensors.CANCoder;
//import com.ctre.phoenix.sensors.CANCoderConfiguration;
//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.MotorType;

//import com.revrobotics.CANSparkFlex.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Flywheel extends SubsystemBase {
  /** Creates a new Flywheel. */

  private CANSparkFlex flywheelMotor;
  
  private CANSparkFlex flywheelIndexerMotor;

  private double setSpeed;
  private double setVelocity;

  private CANCoder flywheelCanCoder;

  public Flywheel() {
    flywheelMotor = new CANSparkFlex(KKFlyWheelUpperMotorID, MotorType.kBrushless);
      flywheelMotor = new CANSparkFlex(KKFlyWheellowerMotorID, MotorType.kBrushless);
      
      flywheelCanCoder = new CANCoder(KFlywheelEncoderID);

    flywheelMotor.setIdleMode(IdleMode.kBrake);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
