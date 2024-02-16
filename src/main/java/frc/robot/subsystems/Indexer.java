// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IndexerConstants.*;


//import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;



public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
 
  private CANSparkMax indexerMotor;
  // private double setSpeed;
  private DigitalInput indexerLimitSwitchNoteMaxPos;
  
  public Indexer() {
    indexerMotor = new CANSparkMax(KIndexerMotorID, MotorType.kBrushless);
    indexerLimitSwitchNoteMaxPos = new DigitalInput(KIndexerLimitSwitchNoteMaxPosID);
  }

  @Override
  public void periodic(
  ) {
    // This method will be called once per scheduler run
  }

  //Indexer Methods
public void indexerSpin(double speed){
    indexerMotor.set(speed);
  //create a speed constant
}

public void indexerStop(){
    indexerMotor.set(0);
  }

public void indexerNoteLoaded(double speed){
    indexerMotor.set(speed);
}

//Limit Switch Methods

 public boolean getIndexerIDLimitSwitch(){
    return indexerLimitSwitchNoteMaxPos.get();
 }

public void IndexerSpinToLS(double indexerSpeed){
    if (indexerSpeed >= 0 && getIndexerIDLimitSwitch()) {
        indexerMotor.set(0);
    }
     else {
      indexerMotor.set(0);
    }
  }
}


