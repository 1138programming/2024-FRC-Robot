// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;


public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  private CANSparkMax indexerMotor;
  
  public Indexer() {

  }

  @Override
  public void periodic(



  ) {
    // This method will be called once per scheduler run
  }
}
