// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import static frc.robot.Constants.LimelightConstants.KlimelightMountHeight;
import static frc.robot.Constants.LimelightConstants.KspeakerHeight;
import static frc.robot.Constants.ShooterTiltConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DutyCycle;
// import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SubsystemUtil;

//import frc.robot.subsystems.Limelight; *SPEAK TO THOMAS!!!*

import com.revrobotics.CANSparkMax; // Covers Neo's
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
// import com.ctre.phoenix6.signals.SensorDirectionValue;

public class ShooterTilt extends SubsystemBase {
  private CANSparkMax shooterTiltMotor;
  private CANcoder shooterTiltCANcoder;
  private DutyCycleEncoder shooterTiltThroughBoreEncoder;
  private double startingAngle;
  private DigitalInput shooterTiltBottomLS;

  // PID
  private PIDController swivelController;
  private PIDController absoluteSwivelController;
  
  private boolean manualControl;

  public ShooterTilt() {
    // Motor Setup
    shooterTiltMotor = new CANSparkMax(KShooterTiltMotorID, MotorType.kBrushless);

    shooterTiltMotor.setIdleMode(IdleMode.kBrake);
    shooterTiltMotor.setInverted(false);

    // CANCoder Setup
    shooterTiltCANcoder = new CANcoder(KShooterTiltEncoderID);

    shooterTiltThroughBoreEncoder = new DutyCycleEncoder(KShooterTiltAbsoluteEncoderID);
    shooterTiltThroughBoreEncoder.reset();
    shooterTiltThroughBoreEncoder.setPositionOffset(KShooterTiltAbsoluteOffset);
    shooterTiltThroughBoreEncoder.setDistancePerRotation(360);

    shooterTiltBottomLS = new DigitalInput(KShooterTiltBottomLimitSwitch);

    absoluteSwivelController = new PIDController(0.4, 0, 0);
    // startingAngle = 202.91833 - shooterTiltCANcoder.getPosition().getValueAsDouble() * 360;

    SmartDashboard.putNumber("absolute P", KShooterTiltAbsoluteControllerP);
    SmartDashboard.putNumber("absolute I", KShooterTiltAbsoluteControllerI);
    SmartDashboard.putNumber("absolute D", KShooterTiltAbsoluteControllerD);

    SmartDashboard.putNumber("Tilt Offset Close", KShooterTiltCloseAimOffset);
    SmartDashboard.putNumber("Tilt Offset Medium", KShooterTiltMediumAimOffset);
    SmartDashboard.putNumber("Tilt Offset Far", KShooterTiltFarAimOffset);

    // swivelshootController = new PIDController(KShooterTiltControllerShootP, 0, 0);
  }
  
  public double getTiltEncoder() {
    // double angle = shooterTiltCANcoder.getPosition().getValueAsDouble() * 360 + startingAngle;
    double angle = 360 - shooterTiltCANcoder.getPosition().getValueAsDouble() * 360;
    return angle;
  }
  public boolean getLS() {
    return !shooterTiltBottomLS.get();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("tilt encoder angle", getTiltEncoder());
    SmartDashboard.putNumber("tilt shooter angle", getShooterAngle());
    SmartDashboard.putNumber("tilt shooter angle through bore", shooterTiltThroughBoreEncoder.getDistance());
    SmartDashboard.putNumber("tilt shooter absolute pos through bore", shooterTiltThroughBoreEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("tilt shooter get final through bore", getAbsoluteEncoder());
    SmartDashboard.putNumber("auto tilt angle", getAngleForShooterPivot(SubsystemUtil.getDistanceFromSpeaker()) + SmartDashboard.getNumber("Tilt Offset", KShooterTiltCloseAimOffset));
    SmartDashboard.putBoolean("manualControl", manualControl);
    SmartDashboard.putBoolean("limitswitch", getLS());


    absoluteSwivelController.setP(SmartDashboard.getNumber("absolute P", KShooterTiltAbsoluteControllerP));
    absoluteSwivelController.setI(SmartDashboard.getNumber("absolute I", KShooterTiltAbsoluteControllerI));
    absoluteSwivelController.setD(SmartDashboard.getNumber("absolute D", KShooterTiltAbsoluteControllerD));
  }

  public void toggleManualControl() {
    manualControl = !manualControl;
  }

  public void moveSwivel(double speed) {
    if (!shooterTiltBottomLS.get() && speed < 0) {
      shooterTiltMotor.set(0);
    }
    else {
      shooterTiltMotor.set(speed);
    }
  }

  // starting angle is 17 degrees and the range is between 17-90 or 15-180 (If i
  // recall correctly)

  public double getAbsoluteEncoder() {
    return shooterTiltThroughBoreEncoder.getDistance() + 80;
  }

  public void swivelToPosAbsolute(double setpoint) {
    if (!manualControl) {
      moveSwivel(absoluteSwivelController.calculate(getAbsoluteEncoder(), setpoint));
    }
  }
  public void swivelToPosAbsoluteFar(double setpoint) {
    if (!manualControl) {
      moveSwivel(absoluteSwivelController.calculate(getAbsoluteEncoder(), setpoint));
    }
  }

  // PID
  public void swivelToPos(double setpoint) {
    if (!manualControl) {
      moveSwivel(swivelController.calculate(getTiltEncoder(), setpoint));
    }
    // moveSwivel(swivelController.calculate(getTiltEncoder(), setPoint) + KShooterTiltAngleOffset);
  }
  public void swivelToPosNoNoteCheck(double setpoint) {
    if (setpoint > 25 && setpoint < 83) {  
      moveSwivel(swivelController.calculate(getTiltEncoder(), setpoint));
    }
    // moveSwivel(swivelController.calculate(getTiltEncoder(), setPoint) + KShooterTiltAngleOffset);
  }

  public static double getMotorAngleFromShooterAngle(double shooterAngle) {
    double motorAngle = 0;
    // shooterAngle += KShooterTiltAngleOffset;

    // Check if the requested shooter angle is in bound, otherwise clamp to max /
    // min value
    if (shooterAngle >= KShooterTiltAngles[0][KShooterTiltAnglesMaxIndex]) {
      motorAngle = KShooterTiltAngles[1][KShooterTiltAnglesMaxIndex];
    } else if (shooterAngle <= KShooterTiltAngles[0][0]) {
      motorAngle = KShooterTiltAngles[1][0];
    }

    // Otherwise we interpolate between values from the lookup table
    else {
      for (int i = 1; i < KShooterTiltAngles[0].length; i++) {
        if (shooterAngle == KShooterTiltAngles[0][i]) {
          motorAngle = KShooterTiltAngles[1][i];
          break;
        } else if (shooterAngle < KShooterTiltAngles[0][i]) {
          motorAngle = SubsystemUtil.lerp(
              shooterAngle,
              KShooterTiltAngles[0][i - 1], KShooterTiltAngles[1][i - 1],
              KShooterTiltAngles[0][i], KShooterTiltAngles[1][i]);
          break;
        }
      }
    }
    return motorAngle / KTiltMotorToSwivelGearRatio;
  }

  public double getShooterAngle() {
    double shooterAngle = 0;
    double motorAngle = getTiltEncoder() * KTiltMotorToSwivelGearRatio;

    // Check if the requested shooter angle is in bound, otherwise clamp to max /
    // min value
    if (motorAngle >= KShooterTiltAngles[1][KShooterTiltAnglesMaxIndex]) {
      shooterAngle = KShooterTiltAngles[0][KShooterTiltAnglesMaxIndex];
    } else if (motorAngle <= KShooterTiltAngles[1][0]) {
      shooterAngle = KShooterTiltAngles[0][0];
    }

    // Otherwise we interpolate between values from the lookup table
    else {
      for (int i = 1; i < KShooterTiltAngles[0].length; i++) {
        if (motorAngle == KShooterTiltAngles[1][i]) {
          shooterAngle = KShooterTiltAngles[0][i];
          break;
        } else if (motorAngle < KShooterTiltAngles[1][i]) {
          shooterAngle = SubsystemUtil.lerp(
              motorAngle,
              KShooterTiltAngles[1][i - 1], KShooterTiltAngles[0][i - 1],
              KShooterTiltAngles[1][i], KShooterTiltAngles[0][i]);
          break;
        }
      }
    }
    return shooterAngle;
  }

  // enter limelight stuff here
  public static double getAngleForShooterPivot(double distanceFromSpeakerMeters) {
    double angle = (Math.atan((KspeakerHeight - KShooterTiltDistanceOffGround) / distanceFromSpeakerMeters));
    angle *= (180/Math.PI);
    if (angle < KShooterTiltBottomAngle) {
      angle = KShooterTiltBottomAngle;
    } else if (angle > KShooterTiltSubAngle) {
      angle = KShooterTiltSubAngle;
    }
    return angle; //Meters
  }
}
