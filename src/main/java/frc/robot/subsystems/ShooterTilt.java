// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ShooterTiltConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


//import frc.robot.subsystems.Limelight; *SPEAK TO THOMAS!!!*

import com.revrobotics.CANSparkMax; // Covers Neo's
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;

public class ShooterTilt extends SubsystemBase {

  private CANSparkMax shooterTiltMotor;
  private CANcoder shooterTiltCANcoder;

  //PID
  private PIDController swivelController;

  public ShooterTilt() {
    shooterTiltMotor = new CANSparkMax(KShooterTiltMotorID, MotorType.kBrushless); 
    shooterTiltCANcoder = new CANcoder(KShooterTiltEncoderID);
    shooterTiltMotor.setIdleMode(IdleMode.kBrake);
    
    swivelController = new PIDController(KshooterTiltControllerkP, KshooterTiltControllerkI, KshooterTiltControllerkD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        SmartDashboard.putNumber("Shooter Tilt CanCoder", getTiltEncoder());
        SmartDashboard.putNumber("Shooter Tilt Raw CanCoder", getTiltEncoderRaw());
  }

  public void spinTiltMotor(double speed){ // spins up
    shooterTiltMotor.set(speed);
  }
  
  //enter limelight stuff here
  
  // starting angle is 15 degrees and the range is between 15-90 or 15-180 (If i recall correctlh)
  public double getTiltEncoder() {
    return getTiltEncoderRaw(); 
  }
  public double getTiltEncoderRaw() {
    return shooterTiltCANcoder.getPosition().getValueAsDouble() * 360;
  }

  // PID
  public void moveSwivel(double speed){
    shooterTiltMotor.set(speed);;
  }

public void swiveToPos(double setPoint){
  moveSwivel(swivelController.calculate(getTiltEncoderRaw(), setPoint));
}
}
