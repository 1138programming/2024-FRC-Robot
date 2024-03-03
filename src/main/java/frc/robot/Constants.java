// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
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
    // Controller Ports (check in Driver Station, IDs may be different for each
    // compStreamDeckuter)
    public static final int KLogitechPort = 0;
    public static final int KXboxPort = 1;
    public static final int KCompStreamDeckPort = 2;
    public static final int KTestingStreamDeckPort = 3;
    public static final int KAutonTestingStreamDeckPort = 4;

    // Deadzone
    public static final double KDeadZone = 0.05;

    // Joystick Axis IDs
    public static final int KLeftXAxis = 0;
    public static final int KLeftYAxis = 1;
    public static final int KRightXAxis = 2;
    public static final int KRightYAxis = 3;

    // Joystick Axis IDs
    public static final int KXboxLeftYAxis = 1;
    public static final int KXboxRightYAxis = 5;
    public static final int KXboxLeftXAxis = 0;
    public static final int KXboxRightXAxis = 4;

    // Logitech Button Constants
    public static final int KLogitechButtonX = 1;
    public static final int KLogitechButtonA = 2;
    public static final int KLogitechButtonB = 3;
    public static final int KLogitechButtonY = 4;
    public static final int KLogitechLeftBumper = 5;
    public static final int KLogitechRightBumper = 6;
    public static final int KLogitechLeftTrigger = 7;
    public static final int KLogitechRightTrigger = 8;

    // Xbox Button Constants
    public static final int KXboxButtonA = 1;
    public static final int KXboxButtonB = 2;
    public static final int KXboxButtonX = 3;
    public static final int KXboxButtonY = 4;
    public static final int KXboxLeftBumper = 5;
    public static final int KXboxRightBumper = 6;
    public static final int KXboxSelectButton = 7;
    public static final int KXboxStartButton = 8;
    public static final int KXboxLeftTrigger = 2;
    public static final int KXboxRightTrigger = 3;
  }

  public static class LimelightConstants {
    public static final double KlimelightMountAngleDegrees = 25.0; // Neeeds to be changed
    public static final double KlimelightMountHeight = 0.508; 
    public static final double KShooterTiltMountHeight = 0.2286; 
    public static final double KspeakerHeight = 2.1082;  
    public static final double[] KSpeakerCoordinatesBlue = new double[] { 0, 5.5474108 }; // (X,Y) of the center
                                                                                          // aprilTag
    public static final double[] KspeakerAprilTagsBlue = new double[] { 7, 8 }; // Right To Left
    public static final double[] KSpeakerCoordinatesRed = new double[] { 16.618, 5.5474108 }; // (X,Y) of the center
                                                                                              // aprilTag
    public static final double[] KspeakerAprilTagsRed = new double[] { 9, 10 }; // Right To Left
    public static final double KlimeLightRotP = 0.0167;
    public static final double KlimeLightRotI = 0;
    public static final double KlimeLightRotD = 0;
    public static final double KlimeLightDriveP = 0;
    public static final double KlimeLightDriveI = 0;
    public static final double KlimeLightDriveD = 0;
    public static final double KLimelightAngleDeadzone = 1;
    public static final double KaprilTagOffset = 20;
    public static final PIDController KlimelightrotControl = new PIDController(KlimeLightRotP, KlimeLightRotI,
    KlimeLightRotD);
    public static final PIDController KBaseController = new PIDController(KlimeLightDriveP, KlimeLightDriveI,
    KlimeLightDriveD);
  }

  public static class LEDConstants
  {
    public static final int KLEDPort = 9; //placeholder
  }

  public static class SwerveDriveConstants {
    // Drive motors
    public static final int KLeftFrontDriveID = 2; // SparkFlex + Vortex
    public static final int KRightFrontDriveID = 4; // SparkFlex + Vortex
    public static final int KLeftBackDriveID = 6; // SparkFlex + Vortex
    public static final int KRightBackDriveID = 8; // SparkFlex + Vortex

    // Angle motors
    public static final int KLeftFrontAngleID = 1; // SparkMax + NEO
    public static final int KRightFrontAngleID = 3; // SparkMax + NEO
    public static final int KLeftBackAngleID = 5; // SparkMax + NEO
    public static final int KRightBackAngleID = 7; // SparkMax + NEO

    // CanCoders
    public static final int KLeftFrontEncoderID = 1;
    public static final int KRightFrontEncoderID = 2;
    public static final int KLeftBackEncoderID = 3;
    public static final int KRightBackEncoderID = 4;

    // Swerve Angle PID
    public static final double KAngleP = 0;
    public static final double KAngleD = 0;

    // Drive Angle PID
    public static final double KDriveP = 0;
    public static final double KDriveD = 0;

    // Swerve Current Limits
    public static final int KDriveMotorCurrentLimit = 40;
    public static final int KAngleMotorCurrentLimit = 30;

    // Motor Info
    public static final double KNeoMaxRPM = 5676;
    public static final double KNeoVortexMaxRPM = 6784;
    public static final int KVortexEncoderTicksPerRevolution = 7168;

    // Robot Specs
    public static final double KDriveMotorGearRatio = 1 / 5.51; // Double Check
    public static final double KWheelDiameterMeters = 0.1016;
    public static final double KDriveMotorRotToMeter = KDriveMotorGearRatio * KWheelDiameterMeters * Math.PI;
    public static final double KDriveMotorRPMToMetersPerSec = KDriveMotorRotToMeter / 60;
    public static final double KPhysicalMaxDriveSpeedMPS = KNeoVortexMaxRPM * KDriveMotorRPMToMetersPerSec;
    public static final double KWheelRadialDistanceFromCenter = 0.377;
    public static final double KWheelDistanceFromCenter = 0.267;

    // Swerve Wheel X and Y Coordinates for Driving
    public static final Translation2d KFrontLeftLocation = new Translation2d(
        KWheelDistanceFromCenter, KWheelDistanceFromCenter);
    public static final Translation2d KFrontRightLocation = new Translation2d(
        KWheelDistanceFromCenter, -KWheelDistanceFromCenter);
    public static final Translation2d KBackLeftLocation = new Translation2d(
        -KWheelDistanceFromCenter, KWheelDistanceFromCenter);
    public static final Translation2d KBackRightLocation = new Translation2d(
        -KWheelDistanceFromCenter, -KWheelDistanceFromCenter);

    // Max Speeds
    public static final double KMaxAcceleration = 8;
    public static final double KMaxAngularSpeed = 3.5;

    // Offsets
    //  Meow (Gray Bot)
    public static final double KFrontLeftOffset = -177.68+60 + 180;
    public static final double KFrontRightOffset = 79.01 +180;
    public static final double KBackLeftOffset = -75.67+180;
    public static final double KBackRightOffset = 144.75+180;

    // Drive Motor Reversals
    public static final boolean KFrontLeftDriveReversed = false;
    public static final boolean KFrontRightDriveReversed = false;
    public static final boolean KBackLeftDriveReversed = false;
    public static final boolean KBackRightDriveReversed = false;

    // Angle Motor Reversals
    public static final boolean KFrontLeftAngleReversed = true;
    public static final boolean KFrontRightAngleReversed = true;
    public static final boolean KBackLeftAngleReversed = true;
    public static final boolean KBackRightAngleReversed = true;

    // Swerve CanCoder Reversals
    public static final boolean KFrontLeftDriveEncoderReversed = false;
    public static final boolean KFrontRightDriveEncoderReversed = false;
    public static final boolean KBackLeftDriveEncoderReversed = false;
    public static final boolean KBackRightDriveEncoderReversed = false;

    // Low and high percent: sets max speed of drivetrain for driver
    public static final double KBaseDriveLowPercent = 0.25;
    public static final double KBaseDriveMidPercent = 0.5;
    public static final double KBaseDriveMaxPercent = 1;
    
    public static final double KBaseRotLowPercent = 0.75;
    public static final double KBaseRotMidPercent = 1;
    public static final double KBaseRotMaxPercent = 1.5;

    public static final double KRotationP = 0;
    public static final double KRotationI = 0;
    public static final double KRotationD = 0;
    
    // Auton Config
    public static final HolonomicPathFollowerConfig KPathFollowerConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(20.0, 1.5, 0), // Translation constants
      new PIDConstants(3.0, 0, 0), // Rotation constants
      KPhysicalMaxDriveSpeedMPS,
      KWheelRadialDistanceFromCenter, // Drive base radius (distance from center to furthest module)
      new ReplanningConfig()
    );
  }

  public static class IntakeConstants {
    // Motor ID
    public static final int KIntakeMotorID = 12;

    // Motor setup
    public static final boolean KIntakeMotorIsInverted = false;
    public static final int KIntakeMotorCurrentLimit = 40;

    // Motor speed
    public static final double KIntakeMotorSpeed = 1;
  }

  public static class ShooterTiltConstants {
    // Motor ID
    public static final int KShooterTiltMotorID = 16;
    // CANCoder ID
    public static final int KShooterTiltEncoderID = 5;
    
    // CANCoder offset
    public static final double KShooterTiltEncoderOffset = -107.841796875;

    // Motor Speed
    public static final double KShooterTiltMotorSpeed = 0.25;

    public static final double KShooterTiltDistanceOffGround = 0.0508;

    // Untuned - PID Constants
    public static final double KShooterTiltControllerP = 0.004;
    public static final double KShooterTiltControllerI = 0;
    public static final double KShooterTiltControllerD = 0;
    
    public static final double KShooterTiltControllerPUp = 0.01; 
    public static final double KShooterTiltControllerShootP = 0.018; // holds tilt in place while shooting
    // Testing
    public static final double kShooterTiltDeadZone = 1.5;
    public static final double kShooterTiltUpPos = 135.17578125;
    public static final double KShooterTiltAmpAngle = 115;
    public static final double KShooterTiltSubAngle = 120;
    public static final double KShooterTiltPodiumAngle = 95;
    public static final double KShooterTiltBottomAngle = 60;

    // This array must be sorted for the shooter tilt functionality to work!!!!
    public static final double[][] KShooterTiltAngles =
    {
      {17.71, 18,   20,   25,    30,    35,    40,   45,     50,    55,    60,     65,  70,     75,     80,     83.92}, 
      {0,     12.6, 33.8, 56.78, 71.37, 83.12, 93.5, 103.14, 112.4, 121.5, 130.64, 140, 149.86, 160.72, 173.93, 196.07}
    };
    public static final int KShooterTiltAnglesMaxIndex = KShooterTiltAngles[0].length - 1;
    
  }
  
  public static class FlywheelConstants{
    // Motor IDs
    public static final int KShooterUpperMotor = 14;
    public static final int KShooterLowerMotor = 15;

    // reversed motor
    public static final boolean KFlywhelUpperMotorReversed = true;

    //Motor speeds
    public static final double KFlywheelFullSpeed = 1; 
    public static final double KFlywheelSpeed = 0.7; 
    public static final double KFlywheelLowSpeed = 0.185;
    
    public static final double KFlywheelVelocity = 5000;
    
    // PID Controller
    public static final double KFlywheelP = 0; 
    public static final double KFlywheelI = 0; 
    public static final double KFlywheelD = 0; 
  }

  public static class IndexerConstants{
    public static final double KIndexerMotorSpeed = 0.7;
    public static final double KIndexerSlowSpeed = 0.32;
    public static final double KIndexerSlowBackwardSpeed = 0.15;
    // public static final double KIndexerSlowSpeed = 0.35;
    public static final double KIndexerFastSpeed = 0.9;

    public static final int KIndexerBBreakerTopID = 0;
    public static final int KIndexerBBreakerBottomID = 9;

    public static final int KIndexerMotorID = 13;
  }

  public static class HangConstants {
    public static final int KHangMotorID = 17;

    public static final int KLaserCanID = 5;

    public static final double KHangMotorSpeedUp = 0.5;
    public static final double KHangMotorSpeedDown = -0.5;

    public static final int KHangLimitSwitchDown = 2;
    public static final int KHangLimitSwitchUp = 3;

    public static final int KHangSetPositionUp = 10;
    public static final int KHangSetPositionDown = -10;

    // Piston Pneumatics double Constants
    public static final int KHangPistonLeftInID = 1;
    public static final int KHangPistonLeftOutID = 2;
    public static final int KHangPistonRightInID = 3;
    public static final int KHangPistonRightOutID = 4;
  }

  public static class TrapConstants{
    public static final int KTrapRollerMotorID = 18;
    public static final int KTrapWristMotorID = 19;
    public static final int KTrapIRID = 2;
    public static final int KPotentiometerID = 1;

    public static final double KTrapRollersForwardSpeed = 0.5;
    public static final double KTrapRollersBackwardSpeed = -0.5;
    public static final double KTrapWristUpSpeed = 0.5;
    public static final double KTrapWristDownSpeed = -0.5;
    
    public static final double KAnalogPotentiometerSensorRange  = 270;
    public static final double KAnalogPotentiometerSensorOffset  = 30;
    public static final double KTrapPotentiometerSetpointFront = 260;
    public static final double KTrapPotentiometerSetpointBack = 40;
    public static final double KTrapPotentiometerSetpoint = 170;

    public static final double trapControllerkP = 0.00028;
    public static final double trapControllerkI = 0.000008;
    public static final double trapControllerkD = 0;
  
  }
}
