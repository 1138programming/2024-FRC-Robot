// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

public class LaserCanTesting extends SubsystemBase {
  /** Creates a new LaserCanTesting. */
  private LaserCan laserCan;
  public LaserCanTesting() {
    laserCan = new LaserCan(0);
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
    SmartDashboard.putNumber("LaserCanDistance", laserCanDistance());
    SmartDashboard.getString("LaserCanDisStatus", laserCanDisStatus());
    laserCanDisStatus();
  }

  public double laserCanDistance(){
    LaserCan.Measurement measurement = laserCan.getMeasurement();
    double distance = measurement.distance_mm;
    return distance;
  }

  public void hangstop(){
    if(laserCanDistance() <= 5){
      // hang.motor stop
    }
  }

  public String laserCanDisStatus(){
    LaserCan.Measurement measurement = laserCan.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return "The target is " + measurement.distance_mm + "mm away!";
    } else {
      return "Oh no! The target is out of range, or we can't get a reliable measurement!";
    }
  }

  
}
