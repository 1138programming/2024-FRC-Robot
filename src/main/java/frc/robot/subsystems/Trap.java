// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.TrapConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;

public class Trap extends SubsystemBase {
  /** Creates a new Trap. */
  // 
  private TalonSRX trapRollerMotor;
  private DigitalInput trapNoteSensor;
  private TalonSRX trapWristMotor;
  private AnalogPotentiometer trapPotentiometer;
  private PIDController swivelController;


  public Trap() {
    trapRollerMotor = new TalonSRX(KTrapRollerMotorID);
    trapNoteSensor = new DigitalInput(KTrapIRID);
    trapWristMotor = new TalonSRX(KTrapWristMotorID);
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
      trapRollerMotor.set(TalonSRXControlMode.PercentOutput, KTrapRollersForwardSpeed);
    }
    else if (getTrapNoteSensor()){
      trapRollerMotor.set(TalonSRXControlMode.PercentOutput, KTrapRollersBackwardSpeed);
    }
    trapRollerMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public void moveWristMotor(double speed){
    trapWristMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public double getPotentiometer(){
    return trapPotentiometer.get();
  }

  public void stopRollers(){
    trapRollerMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public void stopWrist(){
    trapWristMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public void swivelToPos(double setPoint){
    swivelController.calculate(getPotentiometer(), setPoint);
  }
}
