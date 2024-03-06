// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import static frc.robot.Constants.LEDConstants.*;
public class LEDs extends SubsystemBase {
  /** Creates a new LED. */
  AddressableLED LEDStrip;
  AddressableLEDBuffer LEDBuffer;
  public LEDs()
  {
    LEDStrip = new AddressableLED(KLEDPort);
    LEDBuffer = new AddressableLEDBuffer(KLEDBuffer);
    LEDStrip.setLength(LEDBuffer.getLength());
    LEDStrip.setData(LEDBuffer);
  }
  public void setLEDStripColor(int R, int G, int B){
    for (int i = 0; i< LEDBuffer.getLength(); i++){
      LEDBuffer.setRGB(i, R, G, B);
    }
    LEDStrip.start();
    LEDStrip.setData(LEDBuffer);

  }

  // This method set the LED Strip in an everyother color pattern. ex. Orange-Blue-Orange-Blue
  public void LEDOrangeBluePattern(){
    LEDStrip.start();
    for(int i = 0; i < LEDBuffer.getLength(); i++){
      if (i/2 == 0){
        LEDBuffer.setRGB(i, 7, 7, 137);
      }
      else{
        LEDBuffer.setRGB(i, 255, 145, 0);
      }
      LEDStrip.setData(LEDBuffer);
    }
  }

  public void LEDOngoingPattern(){
    LEDStrip.start();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
