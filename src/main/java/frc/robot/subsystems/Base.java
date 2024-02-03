package frc.robot.subsystems;

import static frc.robot.Constants.SwerveDriveConstants.*;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Base extends SubsystemBase {
  private SwerveModule frontLeftModule;
  private SwerveModule frontRightModule;
  private SwerveModule backLeftModule;
  private SwerveModule backRightModule;

  private AHRS gyro;

  private final Field2d m_field = new Field2d();
  
  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;
  private Pose2d pose2d;
  
  private Pose2d pose2da = new Pose2d();
  private Pose2d pose2db = new Pose2d();
  private Pose3d pose3da = new Pose3d();
  private Pose3d pose3db = new Pose3d();

  StructPublisher<Pose2d> publisher2d = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();
  StructArrayPublisher<Pose2d> arrayPublisher2d = NetworkTableInstance.getDefault().getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();
  StructPublisher<Pose3d> publisher3d = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose3d.struct).publish();
  StructArrayPublisher<Pose3d> arrayPublisher3d = NetworkTableInstance.getDefault().getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();
  
  private double driveSpeedFactor;
  private double rotSpeedFactor;

  private boolean defenseMode = false;

  public Base() {
    frontLeftModule = new SwerveModule(
        KFrontLeftAngleMotorID,
        KFrontLeftDriveMotorID,
        KFrontLeftMagEncoderID,
        KFrontLeftOffset,
        KFrontLeftDriveReversed,
        KFrontLeftAngleReversed);
    frontRightModule = new SwerveModule(
        KFrontRightAngleMotorID,
        KFrontRightDriveMotorID,
        KFrontRightMagEncoderID,
        KFrontRightOffset,
        KFrontRightDriveReversed,
        KFrontRightAngleReversed);
    backLeftModule = new SwerveModule(
        KBackLeftAngleMotorID,
        KBackLeftDriveMotorID,
        KBackLeftMagEncoderID,
        KBackLeftOffset,
        KBackLeftDriveReversed,
        KBackLeftAngleReversed);
    backRightModule = new SwerveModule(
        KBackRightAngleMotorID,
        KBackRightDriveMotorID,
        KBackRightMagEncoderID,
        KBackRightOffset,
        KBackRightDriveReversed,
        KBackRightAngleReversed);

    gyro = new AHRS(SPI.Port.kMXP);
    gyro.reset();

    kinematics = new SwerveDriveKinematics(
        KFrontLeftLocation, KFrontRightLocation,
        KBackLeftLocation, KBackRightLocation);
    odometry = new SwerveDriveOdometry(kinematics, getHeading(), getPositions());

    driveSpeedFactor = KBaseDriveMidPercent;
    rotSpeedFactor = KBaseRotMidPercent;

    SmartDashboard.putNumber("X and Y PID", 0);
    SmartDashboard.putNumber("rot P", 0);

    AutoBuilder.configureHolonomic(
        this::getPose2d,
        this::resetPose2d,
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
        this
      );
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double maxDriveSpeedMPS) {
    xSpeed *= maxDriveSpeedMPS;
    ySpeed *= maxDriveSpeedMPS;
    rot *= KMaxAngularSpeed * getRotSpeedFactor();

    // feeding parameter speeds into toSwerveModuleStates to get an array of
    // SwerveModuleState objects
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, KPhysicalMaxDriveSpeedMPS);

    if (defenseMode) {
      lockWheels();
    } else {
      // setting module states, aka moving the motors
      frontLeftModule.setDesiredState(states[0]);
      frontRightModule.setDesiredState(states[1]);
      backLeftModule.setDesiredState(states[2]);
      backRightModule.setDesiredState(states[3]);
    }
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

    // feeding parameter speeds into toSwerveModuleStates to get an array of
    // SwerveModuleState objects
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(targetSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, KPhysicalMaxDriveSpeedMPS);

    frontLeftModule.setDesiredState(states[0]);
    frontRightModule.setDesiredState(states[1]);
    backLeftModule.setDesiredState(states[2]);
    backRightModule.setDesiredState(states[3]);
  }

  public void lockWheels() {
    frontLeftModule.lockWheel();
    frontRightModule.lockWheel();
    backLeftModule.lockWheel();
    backRightModule.lockWheel();
  }

  public void resetPose2d(Pose2d pose2d) {
    odometry.resetPosition(getHeading(), getPositions(), pose2d);
  }

  public Pose2d getPose2d() {
    return odometry.getPoseMeters();
  }

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    backRightModule.setDesiredState(desiredStates[3]);
  }

  public SwerveModuleState getModuleState(SwerveModule module) {
    return new SwerveModuleState(module.getDriveEncoderVel(), module.getAngleR2D());
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    states[0] = getModuleState(frontLeftModule);
    states[1] = getModuleState(frontRightModule);
    states[2] = getModuleState(backLeftModule);
    states[3] = getModuleState(backRightModule);

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
    frontLeftModule.resetRelEncoders();
    frontRightModule.resetRelEncoders();
    backLeftModule.resetRelEncoders();
    backRightModule.resetRelEncoders();
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    positions[0] = frontLeftModule.getPosition();
    positions[1] = frontRightModule.getPosition();
    positions[2] = backLeftModule.getPosition();
    positions[3] = backRightModule.getPosition();

    return positions;
  }

  public void resetPose2d() {
    resetAllRelEncoders();
    pose2d = new Pose2d();
    odometry.resetPosition(getHeading(), getPositions(), pose2d);
  }

  // public void resetPose3d(){
  //   resetAllRelEncoders();
  //   pose3da = new Pose3d();
  //   pose3db = new Pose3d();
  //   odometry.resetPosition(getHeading(), null, pose3da);
  // }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(getHeadingDeg());
  }

  public double getHeadingDeg() {
    return -gyro.getAngle();
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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro", getHeadingDeg());
    SmartDashboard.putString("odometry pose", odometry.getPoseMeters().toString());
    SmartDashboard.putNumber("BackLeftCanCoderPos", backLeftModule.getMagDegRaw());
    SmartDashboard.putNumber("FrontLeftCanCoderPos", frontLeftModule.getMagDegRaw());
    SmartDashboard.putNumber("BackRightCanCoderPos", backRightModule.getMagDegRaw());
    SmartDashboard.putNumber("FrontRightCanCoderPos", frontRightModule.getMagDegRaw());

    SmartDashboard.putData("Field", m_field);
    m_field.setRobotPose(odometry.getPoseMeters());

    publisher2d.set(pose2da);
    arrayPublisher2d.set(new Pose2d[] {pose2da, pose2db});
    publisher3d.set(pose3da);
    arrayPublisher3d.set(new Pose3d[] {pose3da, pose3db});

    odometry.update(getHeading(), getPositions());
    pose2d = odometry.getPoseMeters();
  }
}
