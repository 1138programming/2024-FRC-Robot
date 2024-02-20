// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.LimelightConstants.KlimelightMountHeight;
import static frc.robot.Constants.LimelightConstants.KspeakerHeight;
import static frc.robot.Constants.ShooterTiltConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BaseUtil;

//import frc.robot.subsystems.Limelight; *SPEAK TO THOMAS!!!*

import com.revrobotics.CANSparkMax; // Covers Neo's
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class ShooterTilt extends SubsystemBase {
  private CANSparkMax shooterTiltMotor;
  private CANcoder shooterTiltCANcoder;

  //PID
  private PIDController swivelController;

  public ShooterTilt() {
    // Motor Setup
    shooterTiltMotor = new CANSparkMax(KShooterTiltMotorID, MotorType.kBrushless); 
    
    shooterTiltMotor.setIdleMode(IdleMode.kBrake);
    
    // CANCoder Setup
    shooterTiltCANcoder = new CANcoder(KShooterTiltEncoderID);

    double offsetToRotations = KShooterTiltEncoderOffset/360;

    MagnetSensorConfigs canCoderConfig = new MagnetSensorConfigs();
    canCoderConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    canCoderConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    canCoderConfig.MagnetOffset = offsetToRotations;
    shooterTiltCANcoder.getConfigurator().apply(canCoderConfig);
    
    // PID Controller Setup
    swivelController = new PIDController(KShooterTiltControllerP, KShooterTiltControllerI, KShooterTiltControllerD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      SmartDashboard.putNumber("Shooter Tilt CanCoder", getTiltEncoder());
  }

  public void moveSwivel(double speed){
    shooterTiltMotor.set(speed);
  }
  
  // starting angle is 17 degrees and the range is between 17-90 or 15-180 (If i recall correctlh)
  public double getTiltEncoder() {
    return shooterTiltCANcoder.getPosition().getValueAsDouble() * 360;
  }
  
  // PID
  public void swivelToPos(double setPoint){
    moveSwivel(swivelController.calculate(getTiltEncoder(), setPoint));
  }
  
  //enter limelight stuff here
  public double getAngleForShooterPivot() {
    return Math.atan((KspeakerHeight - KlimelightMountHeight) / BaseUtil.getDistanceFromSpeaker());
  }
}
