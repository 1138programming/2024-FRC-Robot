// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.TrapConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;

public class Trap extends SubsystemBase {
  /** Creates a new Trap. */
  // 
  private CANSparkMax trapRollerMotor;
  private DigitalInput trapNoteSensor;
  private CANSparkMax trapWristMotor;
  private AnalogPotentiometer trapPotentiometer;
  private PIDController swivelController;


  public Trap() {
    trapRollerMotor = new CANSparkMax(KTrapRollerMotorID, MotorType.kBrushless);
    trapNoteSensor = new DigitalInput(KTrapIRID);
    trapWristMotor = new CANSparkMax(KTrapWristMotorID, MotorType.kBrushless);
    trapPotentiometer = new AnalogPotentiometer(KPotentiometerID, KAnalogPotentiometerSensorRange, KAnalogPotentiometerSensorOffset); //Change input later
    swivelController = new PIDController(trapControllerkP, trapControllerkI, trapControllerkD);
  } 

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("sensorstate", getTrapNoteSensor());
    // This method will be called once per scheduler run
  }

  public boolean getTrapNoteSensor(){
    return !trapNoteSensor.get();
  }

  public void moveTrapRollers(double speed){
    if (!getTrapNoteSensor()){
      trapRollerMotor.set(KTrapRollersForwardSpeed);
    }
    else if (getTrapNoteSensor()){
      trapRollerMotor.set(KTrapRollersBackwardSpeed);
    }
    trapRollerMotor.set(0);
  }

  public void moveWristMotor(double speed){
    trapWristMotor.set(speed);
  }

  public double getPotentiometer(){
    return trapPotentiometer.get();
  }

  public void stopRollers(){
    trapRollerMotor.set(0);
  }

  public void stopWrist(){
    trapWristMotor.set(0);
  }

  public void swivelToPos(double setPoint){
    swivelController.calculate(getPotentiometer(), setPoint);
  }
}
