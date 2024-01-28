// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import for talonFX
import com.ctre.phoenix6.*;
import com.ctre.phoenix6.hardware.TalonFX;


public class Intake extends SubsystemBase {
  
  private TalonFX innerIntakeMotor;
  private TalonFX outerIntakeMotor;
  
  /** Creates a new Intake. */
  public Intake() {

    innerIntakeMotor = new TalonFX(KInnerIntakeMotorID);
    outerIntakeMotor = new TalonFX(KOuterIntakeMotorID);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
