// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Hang.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

// Neo Motor sparkmax
// 2 piston pneumatics double solenoid
// an built in encoder and 2 limit switches
// pneumatics are entirely seperate but have the same function
// make sure to add speed constants - don't just name it speed -> scope issues

public class Hang extends SubsystemBase {
  /** Creates a new Hang. */
  private CANSparkMax hangMotor;
  private DigitalInput hangLimitSwitchUp;
  private DigitalInput hangLimitSwitchBottom;

  private DoubleSolenoid hangDoubleSolenoidLeft;
  private DoubleSolenoid hangDoubleSolenoidRight;

  public Hang() {
    hangMotor = new CANSparkMax(KHangMotorID, MotorType.kBrushless);
    hangLimitSwitchUp = new DigitalInput(KHangLimitSwitchUp);
    hangLimitSwitchBottom = new DigitalInput(KHangLimitSwitchDown);
    // create constants for the pneumatics
    //figure out motor type
    hangDoubleSolenoidLeft = new DoubleSolenoid(PneumaticsModuleType.REVPH, KHangDoubleSolenoidLeftInID, KHangDoubleSolenoidLeftOutID);
    hangDoubleSolenoidRight = new DoubleSolenoid(PneumaticsModuleType.REVPH, KHangDoubleSolenoidRightInID, KHangDoubleSolenoidRightOutID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
// add two methods on and off for each pneumatic system
// make them to things in the commands later

  public void hangLeftSolenoidIn(){
    hangDoubleSolenoidLeft.set(DoubleSolenoid.Value.kForward);
  }

  public void hangLeftSolenoidOut(){
    hangDoubleSolenoidLeft.set(DoubleSolenoid.Value.kReverse);
  }

  public void hangRightSolenoidIn(){
    hangDoubleSolenoidLeft.set(DoubleSolenoid.Value.kForward);
  }

  public void hangRightSolenoidOut(){
    hangDoubleSolenoidRight.set(DoubleSolenoid.Value.kReverse);
  }

  public void setHangPosUp(){
    while (hangMotor.getEncoder().getPosition() >=0 && !getHangTopLimitSwitch()){
      hangMotor.getEncoder().setPosition(KHangSetPositionUp);//constant
      hangMotor.set(KHangMotorSpeedUp);//constant
    }
    hangMotor.set(0);
  } 

// Moves hang to a set position
  public void setHangPosDown(){
      while (hangMotor.getEncoder().getPosition() <= 0 && !getHangBottomLimitSwitch()){
        hangMotor.getEncoder().setPosition(KHangSetPositionDown);//constant
        hangMotor.set(KHangMotorSpeedDown);//constant
      }
      hangMotor.set(0);
  }
  
  public double getHangPosition(){
    return hangMotor.getEncoder().getPosition();
  }

  public boolean getHangTopLimitSwitch(){
    return hangLimitSwitchUp.get();
  }

  public boolean getHangBottomLimitSwitch(){
    return hangLimitSwitchBottom.get();
  }
// Only moves up and down to the limit switches
  public void moveHangMotor(double hangSpeed){
    if(hangSpeed <= 0 ){
      if (getHangBottomLimitSwitch()){
        hangMotor.set(0);
      }
    }
    else if(hangSpeed >= 0){
      if (getHangTopLimitSwitch()){
        hangMotor.set(0);
      }
    }
  }

  public void hangStop(){
    hangMotor.set(0);
  }
}
