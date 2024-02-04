// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static frc.robot.Constants.Hang.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.DoubleSolenoid;
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

  private DoubleSolenoid hangPistonLeft;
  private DoubleSolenoid hangPistonRight;

  public Hang() {
    hangMotor = new CANSparkMax(KHangMotorID, MotorType.kBrushless);
    hangLimitSwitchUp = new DigitalInput(KHangLimitSwitchUp);
    hangLimitSwitchBottom = new DigitalInput(KHangLimitSwitchDown);
    // create constants for the pneumatics
    //figure out motor type
    // which one REVPH and CTREPCM
    hangPistonLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, KHangPistonLeftInID, KHangPistonLeftOutID);
    hangPistonLeft.set(DoubleSolenoid.Value.kReverse);
    hangPistonRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, KHangPistonRightInID, KHangPistonRightOutID);
    hangPistonRight.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
// add two methods on and off for each pneumatic system
// make them to things in the commands later
// add pneumatics into methods and commands

  public void setHangHookPosUp(double hangSpeed){

    if (hangMotor.getEncoder().getPosition() >=0 && !getHangTopLimitSwitch()){
      hangMotor.getEncoder().setPosition(KHangSetPositionUp);
      hangMotor.set(KHangMotorSpeedUp);
    }
    hangMotor.set(0);
  } 
  

// Moves hang to a set position

  public void setHangHookPosDown(double hangSpeed){
    if (hangMotor.getEncoder().getPosition() <= 0 && !getHangBottomLimitSwitch()){
      hangMotor.getEncoder().setPosition(KHangSetPositionDown);
      hangMotor.set(KHangMotorSpeedDown);
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

  // Pneumatics
  public void moveHangPistons(){
    hangPistonLeft.toggle();
    hangPistonRight.toggle();
  }

  public void hangStop(double hangSpeed){
    hangMotor.set(0);
  }
}
