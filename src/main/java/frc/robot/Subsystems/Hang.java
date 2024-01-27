// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Hang.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;

// Neo Motor not sure yet for sparkmax
// 2 piston pneumatics
// an built in encoder and 2 limit switches
// Still have to add encoder and the pison pneumatics

public class Hang extends SubsystemBase {
  /** Creates a new Hang. */
  private CANSparkMax hangMotor;
  private DigitalInput hangTopLimitSwitch;
  private DigitalInput hangBottomLimitSwitch;

  public Hang() {
    hangMotor = new CANSparkMax(KHangMotorID, MotorType.kBrushless);
    hangTopLimitSwitch = new DigitalInput(KHangLimitSwitchUp);
    hangBottomLimitSwitch = new DigitalInput(KHangLimitSwitchDown);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setHangPosUp(){
    while (hangMotor.getEncoder().getPosition() >=0 && !getHangTopLimitSwitch()){
      hangMotor.getEncoder().setPosition(10);
    }
  } 

  public void setHangPosDown(){
      while (hangMotor.getEncoder().getPosition() <= 0 && !getHangBottomLimitSwitch()){
        hangMotor.getEncoder().setPosition(-10);
      }
  }
  
  public double getHangPosition(){
    return hangMotor.getEncoder().getPosition();//
  }
  public boolean getHangTopLimitSwitch(){
    return hangTopLimitSwitch.get();
  }
  public boolean getHangBottomLimitSwitch(){
    return hangBottomLimitSwitch.get();
  }

  public void moveHangMotor(double speed){
    hangMotor.set(speed);
    if(speed <= 0 ){
      if (getHangBottomLimitSwitch()){
        hangMotor.set(0);
      }
    }
    else if(speed >= 0){
      if (getHangTopLimitSwitch()){
        hangMotor.set(0);
      }
    }
  }

  public void hangStop(){
    hangMotor.set(0);
  }
}
