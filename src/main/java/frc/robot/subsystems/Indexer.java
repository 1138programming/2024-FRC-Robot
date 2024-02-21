// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IndexerConstants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


//import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  private CANSparkMax indexerMotor;

  private DigitalInput indexerBeamBreakerTop;
  private DigitalInput indexerBeamBreakerBottom;
  
  public Indexer() {
    indexerMotor = new CANSparkMax(KIndexerMotorID, MotorType.kBrushless);

    indexerMotor.setIdleMode(IdleMode.kBrake);
    indexerBeamBreakerTop = new DigitalInput(KIndexerBBreakerNoteMaxPosID);
    indexerBeamBreakerBottom = new DigitalInput(KIndexerBBreakerNoteSlowID);

    
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Indexer Beam Breaker", getIndexerBBreakerTop());
    // This method will be called once per scheduler run

  }

  //Indexer Methods
  public void indexerSpin(double speed){
    indexerMotor.set(speed);
  }

  public void indexerStop(){
    indexerMotor.set(0);
  }

  public void indexerNoteLoaded(double indexerSpeed){
    if (indexerSpeed >= 0 && getIndexerBBreakerTop()) {
        indexerMotor.set(0);
    }
     else {
      indexerMotor.set(indexerSpeed);
    }
  }

  //Limit Switch Methods
  public boolean getIndexerBBreakerTop(){
    return !indexerBeamBreakerTop.get();
  }

  public boolean getIndexerBBreakerBottom(){
    return !indexerBeamBreakerBottom.get();
  }
}