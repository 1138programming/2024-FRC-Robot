// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* 
 * This code is for LaserCan Testing and is not official Hang code.
 * Hang code that is unrelated to the LaserCan may not be fully updated Hang code.
 */

package frc.robot.Subsystems;

import static frc.robot.Constants.Hang.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hang extends SubsystemBase {
  private CANSparkMax hangMotor;
  private DigitalInput hangLimitSwitchUp;
  private DigitalInput hangLimitSwitchBottom;

  private DoubleSolenoid hangPistonLeft;
  private DoubleSolenoid hangPistonRight;
  private LaserCan laserCan;

  public Hang() {
    hangMotor = new CANSparkMax(KHangMotorID, MotorType.kBrushless);
    hangLimitSwitchUp = new DigitalInput(KHangLimitSwitchUp);
    hangLimitSwitchBottom = new DigitalInput(KHangLimitSwitchDown);
    hangPistonLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, KHangPistonLeftInID, KHangPistonLeftOutID);
    hangPistonLeft.set(DoubleSolenoid.Value.kReverse);
    hangPistonRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, KHangPistonRightInID, KHangPistonRightOutID);
    hangPistonRight.set(DoubleSolenoid.Value.kReverse);

    laserCan = new LaserCan(5);
    try {
    laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
    laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(6, 6, 16, 16));
    laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e){
      System.out.println("Configuration failed! " + e);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LaserCanDistanceMM", laserCanDistanceMM());
    SmartDashboard.putNumber("LaserCanDistanceInch", laserCanDistanceInch());
    SmartDashboard.getString("LaserCanDisStatus", laserCanDisStatus());
    laserCanDisStatus();
  }

public double laserCanDistanceMM(){
    LaserCan.Measurement measurement = laserCan.getMeasurement();
    double distance = measurement.distance_mm;
    return distance;
  }

  public double laserCanDistanceInch(){
    LaserCan.Measurement measurement = laserCan.getMeasurement();
    double distance = measurement.distance_mm;
    return distance/25.4;
  }

  public String laserCanDisStatus(){
    LaserCan.Measurement measurement = laserCan.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return "The target is " + measurement.distance_mm + "mm away!";
    } else {
      return "Oh no! The target is out of range, or we can't get a reliable measurement!";
    }
  }

  public void setHangHookPosUp(double hangSpeed){

    if (hangMotor.getEncoder().getPosition() >=0 && !getHangTopLimitSwitch()){
      hangMotor.getEncoder().setPosition(KHangSetPositionUp);
      hangMotor.set(KHangMotorSpeedUp);
    }
    hangMotor.set(0);
  } 

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

  public void moveHangPistons(){
    hangPistonLeft.toggle();
    hangPistonRight.toggle();
  }

  public void hangStop(double hangSpeed){
    hangMotor.set(0);
  }
}
