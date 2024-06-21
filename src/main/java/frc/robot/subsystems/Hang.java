// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.HangConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

// Neo Motor sparkmax
// 2 piston pneumatics double solenoid
// an built in encoder and 2 limit switches
// pneumatics are entirely seperate but have the same function
// make sure to add speed constants - don't just name it speed -> scope issues
// might have to add laser

public class Hang extends SubsystemBase {
  /** Creates a new Hang. */
  private CANSparkFlex rightHangMotor;
  private CANSparkFlex leftHangMotor;
  private DigitalInput hangLimitSwitchUp;

  private DoubleSolenoid hangPistonLeft;
  private DoubleSolenoid hangPistonRight;
  private LaserCan hangLaserCanBottom;
  private Compressor compressor;

  public Hang() {
    // Motor
    rightHangMotor = new CANSparkFlex(KHangMotorID, MotorType.kBrushless);
    leftHangMotor = new CANSparkFlex(18, MotorType.kBrushless);

    // LaserCan
    hangLaserCanBottom = new LaserCan(0); // CHANGE TO PORT
    try {
      hangLaserCanBottom.setRangingMode(LaserCan.RangingMode.SHORT);
      hangLaserCanBottom.setRegionOfInterest(new LaserCan.RegionOfInterest(6, 6, 16, 16));
      hangLaserCanBottom.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
    SmartDashboard.putBoolean("LaserCan Configured?", getlaserCanStatus());
    LaserCan.Measurement measurement = hangLaserCanBottom.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      SmartDashboard.putNumber("Hang Height CM", measurement.distance_mm * 0.1);
    }

    // Pneumatics
    compressor = new Compressor(PneumaticsModuleType.REVPH);
    compressor.enableAnalog(90, 110);

    hangPistonLeft = new DoubleSolenoid(PneumaticsModuleType.REVPH, KHangPistonLeftForwardID,
        KHangPistonLeftBackwardID);
    hangPistonRight = new DoubleSolenoid(PneumaticsModuleType.REVPH, KHangPistonRightBackwardID,
        KHangPistonRightForwardID);

    hangPistonLeft.set(DoubleSolenoid.Value.kForward);
    hangPistonRight.set(DoubleSolenoid.Value.kForward);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pressure", compressor.getPressure());
    LaserCan.Measurement measurement = hangLaserCanBottom.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      SmartDashboard.putNumber("Hang Height CM", measurement.distance_mm * 0.1);
    }
  }

  // add two methods on and off for each pneumatic system
  // make them to things in the commands later
  // add pneumatics into methods and commands

  // Only moves up and down to the limit switches
  public void moveHangMotor(double speed) {
    rightHangMotor.set(speed);
    leftHangMotor.set(speed);
  }

  // Pneumatics
  public void moveHangPistons() {
    hangPistonLeft.toggle();
    hangPistonRight.toggle();
  }

  public void moveHangPistonsUp() {
    hangPistonLeft.set(DoubleSolenoid.Value.kReverse);
    hangPistonRight.set(DoubleSolenoid.Value.kReverse);
  }

  public void moveHangPistonsDown() {
    hangPistonLeft.set(DoubleSolenoid.Value.kForward);
    hangPistonRight.set(DoubleSolenoid.Value.kForward);
  }

  public double getHangDistanceFromBottom() {
    return hangLaserCanBottom.getMeasurement().distance_mm * 0.1; // CM
  }

  public boolean getHangTopLimitSwitch() {
    return true;
    // return hangLimitSwitchUp.get();
  }

  public void hangStop() {
    rightHangMotor.set(0);
    leftHangMotor.set(0);
  }

  public double getHangDistanceFromBottomInch() {
    return getHangDistanceFromBottom() / 0.393701;
  }

  public boolean getlaserCanStatus() {
    LaserCan.Measurement measurement = hangLaserCanBottom.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return true;
    } else {
      return false;
    }
  }
}
