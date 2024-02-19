// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.FlywheelConstants.*;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import com.revrobotics.RelativeEncoder;
//import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import static frc.robot.Constants.FlywheelConstants;
//import com.revrobotics.CANSparkFlex.IdleMode;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Flywheel extends SubsystemBase {
  /** Creates a new Flywheel. */
  
  private CANSparkFlex flywheelUpperMotor;
  
  private CANSparkFlex flywheelLowerMotor;

  private RelativeEncoder upperFlywheelEncoder;
  private RelativeEncoder lowerFlywheelEncoder;

  public Flywheel() {
    flywheelUpperMotor = new CANSparkFlex(KShooterUpperMotor, MotorType.kBrushless);
    flywheelLowerMotor = new CANSparkFlex(KShooterLowerMotor, MotorType.kBrushless);
    
    flywheelUpperMotor.setInverted(true);
    flywheelLowerMotor.setInverted(true);

    flywheelUpperMotor.setIdleMode(IdleMode.kCoast);
    flywheelLowerMotor.setIdleMode(IdleMode.kCoast);

    upperFlywheelEncoder = flywheelUpperMotor.getEncoder();
    lowerFlywheelEncoder = flywheelLowerMotor.getEncoder();
  }

  public void stopMotors(){
    flywheelUpperMotor.set(0);
    flywheelLowerMotor.set(0);
  }
  public void spinFlywheel(double speed){
    flywheelUpperMotor.set(speed);
    flywheelLowerMotor.set(-speed);
  }

  public void spinUpperFlywheel(double speed){
    flywheelUpperMotor.set(speed);
  }

  public void spinLowerFlywheel(double speed){
    flywheelLowerMotor.set(speed);
  }
  // Encoder
  public double getUpperMotorEncoder(){
    return upperFlywheelEncoder.getVelocity();
  }

  public double getLowerMotorEncoder(){
    return lowerFlywheelEncoder.getVelocity();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
