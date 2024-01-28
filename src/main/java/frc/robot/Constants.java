// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

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

  public static class SwerveDriveConstants {
    public static final int KFrontLeftAngleMotorID = 1;  	// SparkMax + NEO
    public static final int KFrontLeftDriveMotorID = 2;  	// SparkMax + NEO
    public static final int KFrontRightAngleMotorID = 3;  	// SparkMax + NEO
    public static final int KFrontRightDriveMotorID = 4;  	// SparkMax + NEO
    public static final int KBackLeftAngleMotorID = 5;  	  // SparkMax + NEO
    public static final int KBackLeftDriveMotorID = 6;  	  // SparkMax + NEO
    public static final int KBackRightAngleMotorID = 7;  	// SparkMax + NEO
    public static final int KBackRightDriveMotorID = 8;  	// SparkMax + NEO

    public static final int KFrontLeftMagEncoderID = 1;
    public static final int KFrontRightMagEncoderID = 2;
    public static final int KBackLeftMagEncoderID = 3;
    public static final int KBackRightMagEncoderID = 4;

    public static final double KAngleP = 0.006;
    public static final double KAngleI = 0;
    public static final double KAngleD = 0;

    public static final int KDriveMotorCurrentLimit = 40;
    public static final int KAngleMotorCurrentLimit = 30;

    private static final double KDriveMotorGearRatio = 1/6.75; // CHANGE FOR ACTUAL ROBOT
    private static final double KWheelDiameterMeters = 0.1016; // CHANGE FOR ACTUAL ROBOT
    public static final double KDriveMotorRotToMeter = KDriveMotorGearRatio * KWheelDiameterMeters * Math.PI;
    public static final double KDriveMotorRPMToMetersPerSec = KDriveMotorRotToMeter / 60;
    public static final double KNeoMaxRPM = 5700;
    public static final double KNeoVortexMaxRPM = 5700;
    public static final double KPhysicalMaxDriveSpeedMPS = KNeoMaxRPM * KDriveMotorRPMToMetersPerSec;

    public static final double KFrontLeftOffset = -12.44;
    public static final double KFrontRightOffset = 311.81;  
    public static final double KBackLeftOffset =  332.5;
    public static final double KBackRightOffset = -17.75;

    public static final boolean KFrontLeftDriveReversed = false;
    public static final boolean KFrontRightDriveReversed = false;
    public static final boolean KBackLeftDriveReversed = false;
    public static final boolean KBackRightDriveReversed = false;
    
    public static final boolean KFrontLeftAngleReversed = true;
    public static final boolean KFrontRightAngleReversed = true;
    public static final boolean KBackLeftAngleReversed = true;
    public static final boolean KBackRightAngleReversed = true;
    
    public static final boolean KFrontLeftDriveEncoderReversed = false;
    public static final boolean KFrontRightDriveEncoderReversed = false;
    public static final boolean KBackLeftDriveEncoderReversed = false;
    public static final boolean KBackRightDriveEncoderReversed = false;
  
    public static final double KGyroOffset = 180;


    public static final double KMaxAcceleration = 4;
  
    // find for testbed
    public static final double KMaxAngularSpeed = 3.5; 
    
      // Low and high percent: sets max speed of drivetrain for driver
    public static final double KBaseDriveLowPercent = 0.25;
    public static final double KBaseDriveMidPercent = 0.5;
    public static final double KBaseDriveMaxPercent = 1;
  
    public static final double KBaseRotLowPercent = 0.75;
    public static final double KBaseRotMidPercent = 1;
    public static final double KBaseRotMaxPercent = 1.5;

      public static final double KWheelDistanceFromCenter = 0.3048;
  public static final Translation2d KFrontLeftLocation = new Translation2d(
    KWheelDistanceFromCenter, KWheelDistanceFromCenter
  );
  public static final Translation2d KFrontRightLocation = new Translation2d(
    KWheelDistanceFromCenter, -KWheelDistanceFromCenter
  );
  public static final Translation2d KBackLeftLocation = new Translation2d(
    -KWheelDistanceFromCenter, KWheelDistanceFromCenter
  );
  public static final Translation2d KBackRightLocation = new Translation2d(
    -KWheelDistanceFromCenter, -KWheelDistanceFromCenter
  );
    public static final int KVortexEncoderTicksPerRevolution = 7168;
  }
}
