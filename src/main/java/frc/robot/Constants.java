// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class TiltConstants{
    
  }
  public static class TrapConstants{
    public static final int KTrapRollerMotorID = 1;
    public static final int KTrapIRSensorID = 6;
    public static final int KTrapWristMotorID = 3;

    public static final double KTrapRollersForwardSpeed = 0.5;
    public static final double KTrapRollersBackwardSpeed = -0.5;
    public static final double KTrapWristUpSpeed = 0.5;
    public static final double KTrapWristDownSpeed = -0.5;
    
    public static final double KAnalogPotentiometerSensorRange  = 270;
    public static final double KAnalogPotentiometerSensorOffset  = 30;

    public static final double trapControllerkP = 0.00028;
    public static final double trapControllerkI = 0.000008;
    public static final double trapControllerkD = 0;
  
  }
}
