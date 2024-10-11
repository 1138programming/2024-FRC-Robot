package frc.robot.subsystems;

import static frc.robot.Constants.SwerveDriveConstants.*;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  private CANSparkMax angleMotor;
  private int driveMotorID;
  // private CANSparkMax driveMotor;
  private CANSparkFlex driveMotor;

  // magEncoder = absolute encoder to reset position of relative angle encoders
  private CANcoder canCoder;

  // Relative encoders are used for robot odometry and controlling speed/position
  private RelativeEncoder driveEncoder;

  private PIDController angleController;
  private PIDController driveController;
  private SparkPIDController drivingPidController;

  private SwerveModulePosition prevPosition;

  private double offset;
  public SwerveModule(int angleMotorID, int driveMotorID, int encoderPort, double offset, 
                      boolean driveMotorReversed, boolean angleMotorReversed) {
    angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
    driveMotor = new CANSparkFlex(driveMotorID, MotorType.kBrushless);
    // driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    
    angleMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setIdleMode(IdleMode.kBrake);
    // if (driveMotorID == 6) {
    //   driveMotor.setIdleMode(IdleMode.kCoast);
    // }
    this.driveMotorID = driveMotorID;
    this.angleMotor.setInverted(angleMotorReversed);
    this.driveMotor.setInverted(driveMotorReversed);
    
    this.driveMotor.setSmartCurrentLimit(KDriveMotorCurrentLimit); // CURRENTLY 85! NEEDS TESTING
    this.angleMotor.setSmartCurrentLimit(KAngleMotorCurrentLimit); // 40

    canCoder = new CANcoder(encoderPort);

    MagnetSensorConfigs canCoderConfig = new MagnetSensorConfigs();

    double offsetToRotations = offset/360;

    canCoderConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    canCoderConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    canCoderConfig.MagnetOffset = offsetToRotations;
    canCoder.getConfigurator().apply(canCoderConfig);

    // driveEncoder = driveMotor.getExternalEncoder(Type.kQuadrature, 1);
    driveEncoder = driveMotor.getEncoder();
    
    driveEncoder.setPositionConversionFactor(KDriveMotorRotToMeter);
    driveEncoder.setVelocityConversionFactor(KDriveMotorRPMToMetersPerSec);

    
    angleController = new PIDController(KAngleP, 0, KAngleD);
    driveController = new PIDController(KAngleP, 0, KAngleD);
    angleController.enableContinuousInput(-180, 180); // Tells PIDController that 180 deg is same in both directions

    drivingPidController = driveMotor.getPIDController();
    drivingPidController.setP(KDrivingPidP);
    drivingPidController.setI(KDrivingPidI);
    drivingPidController.setD(KDrivingPidD);
    // drivingPidController.setFF(1/Neo);
    drivingPidController.setFF(1/KNeoVortexMaxRPM);
    drivingPidController.setOutputRange(-1, 1);
  }
  
  
  public void setDesiredState(SwerveModuleState desiredState) {
    double angleMotorOutput;
    double driveMotorOutput;
    
    Rotation2d currentAngleR2D = getAngleR2D();
    desiredState = SwerveModuleState.optimize(desiredState, currentAngleR2D);
    angleMotorOutput = angleController.calculate(getAngleDeg(), desiredState.angle.getDegrees());
    
    driveMotorOutput = desiredState.speedMetersPerSecond / KPhysicalMaxDriveSpeedMPS;
    
    // driveMotorOutput = desiredState.speedMetersPerSecond;
    // driveMotorOutput = driveController.calculate(getDriveEncoderVel(), desiredState.speedMetersPerSecond) / KPhysicalMaxDriveSpeedMPS;
    
    angleMotor.set(angleMotorOutput);
    driveMotor.set(driveMotorOutput); 
  }

  // public void setDesiredState(SwerveModuleState desiredState) {
  //   double angleMotorOutput;
    
  //   Rotation2d currentAngleR2D = getAngleR2D();
    
  //   // Command driving and turning SPARKS MAX towards their respective setpoints.
  //   desiredState = SwerveModuleState.optimize(desiredState, currentAngleR2D);

  //   drivingPidController.setReference(desiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
  //   angleMotorOutput = angleController.calculate(getAngleDeg(), desiredState.angle.getDegrees());
    
  //   angleMotor.set(angleMotorOutput);
    
    
  //   // Optimize the reference state to avoid spinning further than 90 degrees.
  //   // desiredState = SwerveModuleState.optimize(desiredState, currentAngleR2D);
    
  //   // // Command driving and turning SPARKS MAX towards their respective setpoints.
  //   // angleMotorOutput = angleController.calculate(getAngleDeg(), desiredState.angle.getDegrees());
  //   // angleMotor.set(angleMotorOutput);
    
  //   // drivingPidController.setReference(desiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
  // }
  
  public void lockWheel() {
    double angleMotorOutput;
    if (angleMotor.getDeviceId() == KLeftFrontAngleID || angleMotor.getDeviceId() == KRightBackAngleID) {
      angleMotorOutput = angleController.calculate(getAngleDeg(), 45);
    }
    else {
      angleMotorOutput = angleController.calculate(getAngleDeg(), -45);
    }
    
    angleMotor.set(angleMotorOutput);
    driveMotor.set(0);
  }
  
  public SwerveModulePosition getPosition() {
    SwerveModulePosition position = new SwerveModulePosition(getDriveEncoderPos(), getAngleR2D());
    if (Math.abs(position.distanceMeters) > 50) {
      position = prevPosition;
    }
    else {
      prevPosition = position;
    } 
    return position;
  }
  
  public void stop() {
    driveMotor.set(0);
    angleMotor.set(0);
  }
  
  public void resetRelEncoders() {
    driveEncoder.setPosition(0);
  }
  
  public double getAbsoluteOffset() {
    return offset;
  }
  
  // Drive Encoder getters
  public double getDriveEncoderPos() {
    return driveEncoder.getPosition();
  }
  public double getDriveEncoderVel() {
    return driveEncoder.getVelocity();
  }
  
  // Angle Encoder getters
  public double getMagDegRaw() {
    double pos = canCoder.getAbsolutePosition().getValueAsDouble() * 360;
    return pos;
  }
  public double getAngleDeg() {
    return getMagDegRaw() % 360;
  }

  public Rotation2d getAngleR2D() {
    return Rotation2d.fromDegrees(getAngleDeg()); 
  }

  @Override
  public void periodic() {
        SmartDashboard.putNumber("swerve #" + driveMotorID, driveMotor.getEncoder().getVelocity());

    // if (drivingPidController.getP() != SmartDashboard.getNumber("DrivingPidP", KDrivingPidP)) {
    //   drivingPidController.setP(SmartDashboard.getNumber("DrivingPidP", KDrivingPidP));
    // }
    // if (drivingPidController.getD() != SmartDashboard.getNumber("DrivingPidD", KDrivingPidD)) {
    //   drivingPidController.setD(SmartDashboard.getNumber("DrivingPidD", KDrivingPidD));
    // }
  }
}
