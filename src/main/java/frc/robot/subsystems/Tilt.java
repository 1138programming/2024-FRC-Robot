// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.TiltConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


//import frc.robot.subsystems.Limelight; *SPEAK TO THOMAS!!!*

import com.revrobotics.CANSparkMax; // Covers Neo's
import com.revrobotics.RelativeEncoder; //might not be required
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.core.CoreCANcoder.*;



public class Tilt extends SubsystemBase {
  /** Tilt includes: (this is what design believes)
   *  - 1 Neo
   *  - 1 CANCoder 
   * */
  private CANSparkMax tiltMotor;
  private CANcoder tiltCANcoder;
  
  public Tilt() {
    tiltMotor = new CANSparkMax(KTiltMotorID, MotorType.kBrushless); // IDs not workings even after import
    
    tiltCANcoder = new CANcoder(KTiltEncoderID);
    tiltCANcoder.setPosition(0);
    tiltMotor.setIdleMode(IdleMode.kBrake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        SmartDashboard.putNumber("Tilt Cancoder", getTiltEncoder());
        SmartDashboard.putNumber("Tilt Raw CanCoder", getTiltEncoderRaw());
  }

  public void spinTiltMotorU(double speed){ // spins up
    tiltMotor.set(speed);
  }

  public void spinTiltMotorD(double speed){ // spins down
    tiltMotor.set(-speed);
  }

  // starting angle is 15 degrees and the range is between 15-90 or 15-180 (If i recall correctlh)
  public double getTiltEncoder() {
    return getTiltEncoderRaw() + 15; 
  }
  public double getTiltEncoderRaw() {
    return tiltCANcoder.getPosition().getValueAsDouble() * 360;
  }
}
