// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.wpilibj.TimedRobot;

public class LaserCanTesting extends SubsystemBase {
  /** Creates a new LaserCanTesting. */
  private LaserCan laserCan;
  public LaserCanTesting() {
    laserCan = new LaserCan(0);
  }


  @Override
  public void init() {
    try {
    laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
    laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(6, 6, 16, 16));
    laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e){
      System.out.println("Configuration failed! " + e);
    }
    // This method will be called once per scheduler run
  }

  @Override
  public void periodic() {
    LaserCan.Measurement measurement = laserCan.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      System.out.println("The target is " + measurement.distance_mm + "mm away!");
    } else {
      System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
      // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
    }
  }

  
}
