// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.TrapConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

// import for the PID Controller
import edu.wpi.first.math.controller.PIDController;


// 775 talon srx, potentiometer, ir sensor/ beam breaker

public class Trap extends SubsystemBase {
  /** Creates a new Trap. */
  // 
  private TalonFX trapRollerMotor;
  private DigitalInput trapNoteSensor;
  private CANSparkMax trapWristMotor;
  private AnalogPotentiometer trapPotentiometer;

  private PIDController swivelController;

  // sensorprivate 
  //poteniometer

  public Trap() {
    trapRollerMotor = new TalonFX(KTrapRollerMotorID);
    trapNoteSensor = new DigitalInput(KTrapIRSensorID);
    trapWristMotor = new CANSparkMax(KTrapWristMotorID,MotorType.kBrushless);
    trapPotentiometer = new AnalogPotentiometer(25, KAnalogPotentiometerSensorRange, KAnalogPotentiometerSensorOffset); //Change input later

    swivelController = new PIDController(trapControllerkP, trapControllerkI, trapControllerkD);
  }

    
// neo for the wrist and the rollers are side by side 
// 775 or 550 but i programmed 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public boolean getTrapNoteSensor(){
    return trapNoteSensor.get();
  }

  public void moveTrapRollersBackward(){
    while (!getTrapNoteSensor()){
      trapRollerMotor.set(KTrapRollersBackwardSpeed);
    }
    trapRollerMotor.set(0);
  }

  public void moveTrapRollersForward(){
    trapRollerMotor.set(KTrapRollersForwardSpeed);
    // add a time for how long the rollers need to 
  }

  public void moveWristMotorForward(){
    trapWristMotor.set(KTrapWristUpSpeed);
  }

  public void moveWristMotorReverse(){
    trapWristMotor.set(KTrapWristDownSpeed);
    //figure out values later
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

  // methods for PID Controller
  public void swivelToPos(double setPoint){
    swivelController.calculate(getPotentiometer(), setPoint);
  }

  
  //create method for set positions for the wrist using potentiometer

}
