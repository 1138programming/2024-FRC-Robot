// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static frc.robot.Constants.UnderIntakeConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import for talonFX
import com.ctre.phoenix6.*;
import com.ctre.phoenix6.hardware.TalonFX;


public class UnderIntake extends SubsystemBase {
  private TalonFX underIntakeMotor;
  /** Creates a new Intake. */
  public UnderIntake() {
    underIntakeMotor = new TalonFX(KUnderIntakeMotorID);
  }
  
  public void underIntakeSpinMotor(double speed) {
    underIntakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
