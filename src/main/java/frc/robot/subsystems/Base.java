package frc.robot.subsystems;

import static frc.robot.Constants.LimelightConstants.KSpeakerCoordinatesBlue;
import static frc.robot.Constants.LimelightConstants.KSpeakerCoordinatesRed;
import static frc.robot.Constants.ShooterTiltConstants.KShooterTiltSubAngle;
import static frc.robot.Constants.SwerveDriveConstants.*;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;


import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SubsystemUtil;

public class Base extends SubsystemBase {
  private SwerveModule leftFrontModule;
  private SwerveModule rightFrontModule;
  private SwerveModule leftBackModule;
  private SwerveModule rightBackModule;

  private AHRS gyro;

  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;

  private double driveSpeedFactor;
  private double rotSpeedFactor;

  private boolean defenseMode = false;
  private Pose2d pose;

  private Limelight limelight;

  private SwerveDrivePoseEstimator poseEstimate;

  public Base() {
    limelight = new Limelight();
    // poseEstimate = new PoseEstimator<>(kinematics, odometry,
    // poseEstimate = new SwerveDrivePoseEstimator

    leftFrontModule = new SwerveModule(
        KLeftFrontAngleID,
        KLeftFrontDriveID,
        KLeftFrontEncoderID,
        KFrontLeftOffset,
        KFrontLeftDriveReversed,
        KFrontLeftAngleReversed);
    rightFrontModule = new SwerveModule(
        KRightFrontAngleID,
        KRightFrontDriveID,
        KRightFrontEncoderID,
        KFrontRightOffset,
        KFrontRightDriveReversed,
        KFrontRightAngleReversed);
    leftBackModule = new SwerveModule(
        KLeftBackAngleID,
        KLeftBackDriveID,
        KLeftBackEncoderID,
        KBackLeftOffset,
        KBackLeftDriveReversed,
        KBackLeftAngleReversed);
    rightBackModule = new SwerveModule(
        KRightBackAngleID,
        KRightBackDriveID,
        KRightBackEncoderID,
        KBackRightOffset,
        KBackRightDriveReversed,
        KBackRightAngleReversed);

    gyro = new AHRS(SPI.Port.kMXP);
    gyro.reset();

    kinematics = new SwerveDriveKinematics(
        KFrontLeftLocation, KFrontRightLocation,
        KBackLeftLocation, KBackRightLocation);
    odometry = new SwerveDriveOdometry(kinematics, getHeading(), getPositions());
    pose = new Pose2d(limelight.getBotPoseX(), limelight.getBotPoseY(), getHeading());
    poseEstimate = new SwerveDrivePoseEstimator(kinematics, getHeading(), getPositions(), pose);

    driveSpeedFactor = KBaseDriveMidPercent;
    rotSpeedFactor = KBaseRotMidPercent;

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetPose,
        this::getSpeeds,
        this::driveRobotRelative,
        KPathFollowerConfig,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);

        // SmartDashboard.putNumber("DrivingPidP", KDrivingPidP);
        // SmartDashboard.putNumber("DrivingPidI", KDrivingPidI);
        // SmartDashboard.putNumber("DrivingPidD", KDrivingPidD);
        SmartDashboard.putNumber("RotP", KRotationP);
      SmartDashboard.putNumber("RotI", KRotationI);
      SmartDashboard.putNumber("RotD", KRotationD);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double maxDriveSpeedMPS,
      double maxRotSpeed) {
    xSpeed *= maxDriveSpeedMPS * getDriveSpeedFactor();
    ySpeed *= maxDriveSpeedMPS * getDriveSpeedFactor();
    rot *= KMaxAngularSpeed * getRotSpeedFactor();
    // rot *= KMaxAngularSpeed * getRotSpeedFactor();

    // feeding parameter speeds into toSwerveModuleStates to get an array of
    // SwerveModuleState objects
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getDrivingHeading())
            // ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading())
            : ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, new Rotation2d()));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, KPhysicalMaxDriveSpeedMPS);

    leftFrontModule.setDesiredState(states[0]);
    rightFrontModule.setDesiredState(states[1]);
    leftBackModule.setDesiredState(states[2]);
    rightBackModule.setDesiredState(states[3]);
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

    // feeding parameter speeds into toSwerveModuleStates to get an array of
    // SwerveModuleState objects
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(targetSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, KPhysicalMaxDriveSpeedMPS);

    leftFrontModule.setDesiredState(states[0]);
    rightFrontModule.setDesiredState(states[1]);
    leftBackModule.setDesiredState(states[2]);
    rightBackModule.setDesiredState(states[3]);
  }

  public void lockWheels() {
    leftFrontModule.lockWheel();
    rightFrontModule.lockWheel();
    leftBackModule.lockWheel();
    rightBackModule.lockWheel();
  }

  public void resetPose(Pose2d pose) {
    poseEstimate.resetPosition(getHeading(), getPositions(), pose);
    odometry.resetPosition(getHeading(), getPositions(), pose);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public Pose2d getPoseEstimate() {
    return poseEstimate.getEstimatedPosition();
  }

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    leftFrontModule.setDesiredState(desiredStates[0]);
    rightFrontModule.setDesiredState(desiredStates[1]);
    leftBackModule.setDesiredState(desiredStates[2]);
    rightBackModule.setDesiredState(desiredStates[3]);
  }

  public SwerveModuleState getModuleState(SwerveModule module) {
    return new SwerveModuleState(module.getDriveEncoderVel(), module.getAngleR2D());
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    states[0] = getModuleState(leftFrontModule);
    states[1] = getModuleState(rightFrontModule);
    states[2] = getModuleState(leftBackModule);
    states[3] = getModuleState(rightBackModule);

    return states;
  }

  // recalibrates gyro offset
  public void resetGyro() {
    gyro.reset();
    gyro.setAngleAdjustment(0);
  }

  public void resetGyro(double gyroOffset) {
    gyro.reset();
    gyro.setAngleAdjustment(gyroOffset);
  }

  public void resetAllRelEncoders() {
    leftFrontModule.resetRelEncoders();
    rightFrontModule.resetRelEncoders();
    leftBackModule.resetRelEncoders();
    rightBackModule.resetRelEncoders();
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    positions[0] = leftFrontModule.getPosition();
    positions[1] = rightFrontModule.getPosition();
    positions[2] = leftBackModule.getPosition();
    positions[3] = rightBackModule.getPosition();

    return positions;
  }

  public double getRobotPoseX() {
    return poseEstimate.getEstimatedPosition().getX();
  }

  public double getRobotPoseY() {
    return poseEstimate.getEstimatedPosition().getY();
  }

  public void resetPose() {
    resetAllRelEncoders();
    pose = new Pose2d();
    odometry.resetPosition(getHeading(), getPositions(), pose);
    poseEstimate.resetPosition(getHeading(), getPositions(), pose);
  }

  public void updatePose(Double x, Double y) {
    pose = new Pose2d(x, y, gyro.getRotation2d());
    odometry.resetPosition(getHeading(), getPositions(), pose);
    poseEstimate.resetPosition(getHeading(), getPositions(), pose);
  }

  public Rotation2d getHeading() {
    double heading = getAimingHeadingDeg();

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        if (heading < 0) {
          heading += 180;
        } else {
          heading -= 180;
        }
      }
    }
    return Rotation2d.fromDegrees(heading);

    // return gyro.getRotation2d(); // TEST
  }
  public Rotation2d getDrivingHeading() {
    double heading = getAimingHeadingDeg();

    return Rotation2d.fromDegrees(heading);

    // return gyro.getRotation2d(); // TEST
  }

  public double getHeadingDeg() {
    return -gyro.getAngle() % 360;
  }

  public double getPositiveHeadingDeg() {
    double angle = getHeadingDeg();
    if (angle < 0) {
      return 360 + angle;
    }
    return angle;
  }

  public double getAimingHeadingDeg() {
    double angle = getHeadingDeg();
    if (angle < -180) {
      angle += 360;
    }
    if (angle > 180) {
      angle -= 360;
    }
    return angle;
  }

  public double getRoll() {
    return gyro.getRoll();
  }

  public double getPitch() {
    return gyro.getPitch();
  }

  public double getDriveSpeedFactor() {
    return driveSpeedFactor;
  }

  public void setDriveSpeedFactor(double speedFactor) {
    driveSpeedFactor = speedFactor;
  }

  public double getRotSpeedFactor() {
    return rotSpeedFactor;
  }

  public void setRotSpeedFactor(double speedFactor) {
    rotSpeedFactor = speedFactor;
  }

  public boolean getDefenseMode() {
    return defenseMode;
  }

  public void setDefenseMode(boolean defenseMode) {
    this.defenseMode = defenseMode;
  }

  public double getDistanceFromSpeaker() {
    double distanceFromSpeaker = 0;
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        double botPoseXOffsetFromSpeaker = poseEstimate.getEstimatedPosition().getX() - KSpeakerCoordinatesBlue[0];
        double botPoseYOffsetFromSpeaker = poseEstimate.getEstimatedPosition().getY() - KSpeakerCoordinatesBlue[1];
        distanceFromSpeaker = Math
            .sqrt(Math.pow(botPoseXOffsetFromSpeaker, 2) + Math.pow(botPoseYOffsetFromSpeaker, 2));
      } else {
        double botPoseXOffsetFromSpeaker = poseEstimate.getEstimatedPosition().getX() - KSpeakerCoordinatesRed[0];
        double botPoseYOffsetFromSpeaker = poseEstimate.getEstimatedPosition().getY() - KSpeakerCoordinatesRed[1];
        distanceFromSpeaker = Math
            .sqrt(Math.pow(botPoseXOffsetFromSpeaker, 2) + Math.pow(botPoseYOffsetFromSpeaker, 2));
      }
    }
    return distanceFromSpeaker;
  }

  public double getAngleFromSpeaker() {
    SmartDashboard.putNumber("gyro input", getPositiveHeadingDeg());
    if (DriverStation.getAlliance().isPresent()) {
      return getAngleFromSpeaker(DriverStation.getAlliance().get(), getRobotPoseX(), getRobotPoseY(), getPositiveHeadingDeg());
    }
    return 0;
  }

  public static double getAngleFromSpeaker(DriverStation.Alliance allianceColor, double xPos, double yPos, double lambda) {
    double theta = 0;
    double angle = 0;
    if (allianceColor == DriverStation.Alliance.Blue) {
      if (Math.abs(xPos - KSpeakerCoordinatesBlue[0]) < 0.01) {
        return 0;
      }
      theta = (Math.atan((yPos - KSpeakerCoordinatesBlue[1]) / (xPos - KSpeakerCoordinatesBlue[0])) * (180/Math.PI));
      angle = theta;
      }
      else {
        if (Math.abs(xPos - KSpeakerCoordinatesRed[0]) < 0.01) {
          return 0;
        }
        theta = (Math.atan((yPos - KSpeakerCoordinatesRed[1]) / (xPos - KSpeakerCoordinatesRed[0])) * (180/Math.PI));
        angle = theta;
    } 
    return angle;
    // return angle;
  }

  public double getAprilTagOffsetFromSpeaker() {
    double offset = 0;

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if(alliance.get() == DriverStation.Alliance.Blue) {
        if (limelight.getTargetFound() && limelight.getTID() == 8) {
          offset = limelight.getXAngle();
        } 
      }
      else if (alliance.get() == DriverStation.Alliance.Red) {
        if (limelight.getTargetFound() && limelight.getTID() == 10) {
          offset = limelight.getXAngle();
        }
      }
    }
    return offset;
  }

  @Override
  public void periodic() {
    // Position Updates
    pose = new Pose2d(limelight.getBotPoseX(), limelight.getBotPoseY(), getHeading());
    if (limelight.getTargetFound()) {
      poseEstimate.addVisionMeasurement(pose, Timer.getFPGATimestamp() - (limelight.getBotPose(5) / 1000));
    }
    poseEstimate.update(getHeading(), getPositions());
    odometry.update(getHeading(), getPositions());
    SmartDashboard.putBoolean("limelight.getTargetFound()", limelight.getTargetFound());

    SubsystemUtil.setDistanceFromSpeaker(getDistanceFromSpeaker());

    // Position Data SmartDashboard
    SmartDashboard.putNumber("Gyro", getHeadingDeg());
    SmartDashboard.putString("odometry pose", odometry.getPoseMeters().toString());
    SmartDashboard.putString("Pose Estimate", poseEstimate.getEstimatedPosition().toString());
    SmartDashboard.putBoolean("getTargetFound", limelight.getTargetFound());
    SmartDashboard.putNumber("getBotPose", limelight.getBotPose(5));
    SmartDashboard.putNumber("Distance from speaker", getDistanceFromSpeaker());
    
    // SwerveDrive Cancoder Position
    SmartDashboard.putNumber("BackLeftCanCoderPos", leftBackModule.getMagDegRaw());
    SmartDashboard.putNumber("FrontLeftCanCoderPos", leftFrontModule.getMagDegRaw());
    SmartDashboard.putNumber("BackRightCanCoderPos", rightBackModule.getMagDegRaw());
    SmartDashboard.putNumber("FrontRightCanCoderPos", rightFrontModule.getMagDegRaw());
    
    // SmartDashboard.putNumber("FrontLeftVel", leftFrontModule.getDriveEncoderVel());
    // SmartDashboard.putNumber("BackLeftVel", leftBackModule.getDriveEncoderVel());
    // SmartDashboard.putNumber("FrontRightVel", rightFrontModule.getDriveEncoderVel());
    // SmartDashboard.putNumber("BackRightVel", rightBackModule.getDriveEncoderVel());
    
    // SmartDashboard.putNumber("lime
    // light botpose", limelight.getBotPose(5));
  }
}
