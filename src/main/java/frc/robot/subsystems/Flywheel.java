// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.FlywheelConstants.*;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private SparkPIDController flywheelUpperController;
  private SparkPIDController flywheelLowerController;

  public Flywheel() {
    flywheelUpperMotor = new CANSparkFlex(KShooterUpperMotor, MotorType.kBrushless);
    flywheelLowerMotor = new CANSparkFlex(KShooterLowerMotor, MotorType.kBrushless);
    
    flywheelUpperMotor.setInverted(true);
    flywheelLowerMotor.setInverted(false);

    flywheelUpperMotor.setIdleMode(IdleMode.kCoast);
    flywheelLowerMotor.setIdleMode(IdleMode.kCoast);

    upperFlywheelEncoder = flywheelUpperMotor.getEncoder();
    lowerFlywheelEncoder = flywheelLowerMotor.getEncoder();
    
    flywheelUpperController = flywheelUpperMotor.getPIDController();
    flywheelUpperController.setP(KFlywheelP);
    flywheelUpperController.setI(KFlywheelI);
    flywheelUpperController.setD(KFlywheelD);
    
    flywheelLowerController = flywheelLowerMotor.getPIDController();
    flywheelLowerController.setP(KFlywheelP);
    flywheelLowerController.setI(KFlywheelI);
    flywheelLowerController.setD(KFlywheelD);

    SmartDashboard.putNumber("F SPEED", 0.175);
    

    // SmartDashboard.putNumber("Flywheel P", 0);
    // SmartDashboard.putNumber("Flywheel I", 0);
    // SmartDashboard.putNumber("Flywheel D", 0);
  }

  public void stopMotors(){
    flywheelUpperMotor.set(0);
    flywheelLowerMotor.set(0);
  }
  public void spinFlywheel(double speed){
    flywheelUpperMotor.set(speed);
    flywheelLowerMotor.set(speed);
  }

  public void setFlywheelVelocity(double velocity) {
    flywheelUpperController.setReference(velocity, ControlType.kVelocity);
    flywheelLowerController.setReference(velocity, ControlType.kVelocity);
  }

  public void spinUpperFlywheel(double speed){
    flywheelUpperMotor.set(speed);
  }

  public void spinLowerFlywheel(double speed){
    flywheelLowerMotor.set(speed);
  }
  // Encoder
  public double getUpperMotorVelocity(){
    return upperFlywheelEncoder.getVelocity();
  }

  public double getLowerMotorVelocity(){
    return lowerFlywheelEncoder.getVelocity();
  }
  
  @Override
  public void periodic() {
    // SmartDashboard.putNumber("flywheel upper speed", getUpperMotorVelocity());
    // SmartDashboard.putNumber("flywheel lower speed", getLowerMotorVelocity());

    // if (SmartDashboard.getNumber("Flywheel P", 0) != flywheelUpperController.getP()) {
    //   flywheelUpperController.setP(SmartDashboard.getNumber("Flywheel P", 0));
    // }
    // if (SmartDashboard.getNumber("Flywheel I", 0) != flywheelUpperController.getI()) {
    //   flywheelUpperController.setI(SmartDashboard.getNumber("Flywheel P", 0));
    // }
    // if (SmartDashboard.getNumber("Flywheel D", 0) != flywheelUpperController.getD()) {
    //   flywheelUpperController.setD(SmartDashboard.getNumber("Flywheel P", 0));
    // }

    // if (SmartDashboard.getNumber("Flywheel P", 0) != flywheelLowerController.getP()) {
    //   flywheelLowerController.setP(SmartDashboard.getNumber("Flywheel P", 0));
    // }
    // if (SmartDashboard.getNumber("Flywheel I", 0) != flywheelLowerController.getI()) {
    //   flywheelLowerController.setI(SmartDashboard.getNumber("Flywheel P", 0));
    // }
    // if (SmartDashboard.getNumber("Flywheel D", 0) != flywheelLowerController.getD()) {
    //   flywheelLowerController.setD(SmartDashboard.getNumber("Flywheel P", 0));
    // }
    
    // This method will be called once per scheduler run
  }
}
