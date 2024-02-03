// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
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
        public static final double KAngleP = 0.006;
        public static final double KAngleI = 0;
        public static final double KAngleD = 0;

        // Swerve Current Limits
        public static final int KDriveMotorCurrentLimit = 40;
        public static final int KAngleMotorCurrentLimit = 30;

        // Motor Info
        public static final double KNeoMaxRPM = 5676;
        public static final double KNeoVortexMaxRPM = 6784;
        public static final int KVortexEncoderTicksPerRevolution = 7168;

        // Robot Specs
        private static final double KDriveMotorGearRatio = 1 / 4.41;
        private static final double KWheelDiameterMeters = 0.1016;
        public static final double KDriveMotorRotToMeter = KDriveMotorGearRatio * KWheelDiameterMeters * Math.PI;
        public static final double KDriveMotorRPMToMetersPerSec = KDriveMotorRotToMeter / 60;
        public static final double KPhysicalMaxDriveSpeedMPS = KNeoVortexMaxRPM * KDriveMotorRPMToMetersPerSec;
        public static final double KWheelRadialDistanceFromCenter = 0.381635;
        public static final double KWheelDistanceFromCenter = 0.269875;

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
        public static final double KFrontLeftOffset = -12.44;
        public static final double KFrontRightOffset = 311.81;
        public static final double KBackLeftOffset = 332.5;
        public static final double KBackRightOffset = -17.75;
        public static final double KGyroOffset = 180;

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

        // Auton Config
        public static final HolonomicPathFollowerConfig KPathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0, 0), // Translation constants
                new PIDConstants(5.0, 0, 0), // Rotation constants
                KPhysicalMaxDriveSpeedMPS,
                KWheelRadialDistanceFromCenter, // Drive base radius (distance from center to furthest module)
                new ReplanningConfig());

        // Possibly Unused
        public static final TrajectoryConfig KtrajectoryConfig = new TrajectoryConfig(KPhysicalMaxDriveSpeedMPS,
                KMaxAcceleration);
    }
}
