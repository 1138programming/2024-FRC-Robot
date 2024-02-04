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
public void IndexerSpin(double speed){
    indexerMotor.set(KIndexerMotorSpeedForward);
    indexerMotor.set(speed);
  //create a speed constant
  }
public void indexerSpin(double speed){
  indexerMotor.set(speed);
}

public void IndexerStop(){
    indexerMotor.set(0);
  }


//Limit Switch Methods
  public boolean getIndexerLimitSwitchNoteMaxPos(){
    return indexerLimitSwitchNoteMaxPos.get();
  }

 public boolean getIndexerIDLimitSwitch(){
    return indexerLimitSwitchNoteMaxPos.get();
 }
public void notesLoaded(double indexerSpeed){
    if (indexerSpeed >= 0 && getIndexerIDLimitSwitch()) {
        indexerMotor.set(0);
    }
  }

}

