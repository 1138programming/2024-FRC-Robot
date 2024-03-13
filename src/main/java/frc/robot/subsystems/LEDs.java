// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;

import static frc.robot.Constants.LEDConstants.*;

public class LEDs extends SubsystemBase {
  /** Creates a new LED. */
  private AddressableLED LEDStrip;
  private AddressableLEDBuffer LEDBuffer;

  private boolean isGreen = false;

  public LEDs()
  {
    LEDStrip = new AddressableLED(KLEDPort);

    LEDBuffer = new AddressableLEDBuffer(KLEDBuffer);
    LEDStrip.setLength(LEDBuffer.getLength());

    LEDStrip.setData(LEDBuffer);
    LEDStrip.start();
  }
  
  public void setLEDStripColor(int R, int G, int B){
    for (int i = 0; i< LEDBuffer.getLength(); i++){
      LEDBuffer.setRGB(i, R, G, B);
    }
    LEDStrip.setData(LEDBuffer);
  }
  
  public void setLEDStripGreen() {
    isGreen = true;
    setLEDStripColor(0, 255, 0);
  }
  public void setLEDStripRed() {
    isGreen = false;
    setLEDStripColor(255, 0, 0);
  }
  public void setLEDStripYellow() {
    setLEDStripColor(255, 255, 0);
  }

  public boolean isGreen() {
    return isGreen();
  }
  
  // This method set the LED Strip in an everyother color pattern. ex. Orange-Blue-Orange-Blue
  public void LEDOrangeBluePattern(){
    for(int i = 0; i < LEDBuffer.getLength(); i++){
      if (i%2 == 0){
        LEDBuffer.setRGB(i, 255, 100, 0);
      }
      else{
        LEDBuffer.setRGB(i, 7, 7, 137);
      }
    }
    LEDStrip.setData(LEDBuffer);
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      LEDOrangeBluePattern();
    }
    
    // This method will be called once per scheduler run
    
  }
}
