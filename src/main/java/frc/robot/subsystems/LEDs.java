// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import static frc.robot.Constants.LEDConstants.*;
public class LEDs extends SubsystemBase {
  /** Creates a new LED. */
  AddressableLED LEDStrip;
  public LEDs()
  {
    LEDStrip = new AddressableLED(KLEDPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
