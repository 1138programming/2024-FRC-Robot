// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.LimelightConstants.KlimelightMountHeight;
import static frc.robot.Constants.LimelightConstants.KspeakerHeight;
import static frc.robot.Constants.ShooterTiltConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SubsystemUtil;

//import frc.robot.subsystems.Limelight; *SPEAK TO THOMAS!!!*

import com.revrobotics.CANSparkMax; // Covers Neo's
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class ShooterTilt extends SubsystemBase {
  private CANSparkMax shooterTiltMotor;
  private CANcoder shooterTiltCANcoder;
  private double startingAngle;

  // PID
  private PIDController swivelController;
  // private PIDController swivelUpController;
  private PIDController swivelshootController;
  
  private boolean manualControl;

  public ShooterTilt() {
    // Motor Setup
    shooterTiltMotor = new CANSparkMax(KShooterTiltMotorID, MotorType.kBrushless);

    shooterTiltMotor.setIdleMode(IdleMode.kBrake);
    shooterTiltMotor.setInverted(true);

    // CANCoder Setup
    shooterTiltCANcoder = new CANcoder(KShooterTiltEncoderID);

    double offsetToRotations = 0;
    // double offsetToRotations = KShooterTiltEncoderOffset / 360;

    MagnetSensorConfigs canCoderConfig = new MagnetSensorConfigs();

    canCoderConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    canCoderConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    canCoderConfig.MagnetOffset = offsetToRotations;
    shooterTiltCANcoder.getConfigurator().apply(canCoderConfig);
    manualControl = false;

    // PID Controller Setups
    swivelController = new PIDController(KShooterTiltControllerP, KShooterTiltControllerI, KShooterTiltControllerD);
    startingAngle = 202.91833 - shooterTiltCANcoder.getPosition().getValueAsDouble() * 360;

    // swivelshootController = new PIDController(KShooterTiltControllerShootP, 0, 0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("tilt encoder angle", getTiltEncoder());
    SmartDashboard.putNumber("tilt shooter angle", getShooterAngle());
    SmartDashboard.putBoolean("manualControl", manualControl);
  }

  public void toggleManualControl() {
    manualControl = !manualControl;
  }

  public void moveSwivel(double speed) {
    shooterTiltMotor.set(speed);
  }

  // starting angle is 17 degrees and the range is between 17-90 or 15-180 (If i
  // recall correctly)
  public double getTiltEncoder() {
    double angle = shooterTiltCANcoder.getPosition().getValueAsDouble() * 360 + startingAngle;
    // if (angle < 0) {
    //   return 360 + angle;
    // }
    return angle;
  }

  // PID
  public void swivelToPos(double setPoint) {
    if (!manualControl) {
      moveSwivel(swivelController.calculate(getTiltEncoder(), setPoint));
    }
    // moveSwivel(swivelController.calculate(getTiltEncoder(), setPoint) + KShooterTiltAngleOffset);
  }

  // public void swivelToPos(double setPoint) {
  //   if (setPoint > getTiltEncoder()) {
  //     moveSwivel(swivelUpController.calculate(getTiltEncoder(), setPoint));
  //   } else {
  //     moveSwivel(swivelController.calculate(getTiltEncoder(), setPoint));
  //   }
  // }

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
    return (Math.atan((KspeakerHeight - KShooterTiltDistanceOffGround) / distanceFromSpeakerMeters) * (180 / Math.PI)); //Meters
  }
}
