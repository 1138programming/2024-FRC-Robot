// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

//import com.revrobotics.CANSparkFlex.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.DigitalInput;

public class Flywheel extends SubsystemBase {
  /** Creates a new Flywheel. */
  
  private CANSparkFlex flywheelUpperMotor;
  
  private CANSparkFlex flywheelLowerMotor;

  private DigitalInput 


 // private double setSpeed;
  //private double flywheelEncoder;


  public Flywheel() {

    flywheelForwardMotor = new CANSparkFlex(KFlyWheelUpperMotorID, MotorType.kBrushless);


    RelativeEncoder flywheelEncoder = flywheelMotor.getEncoder();

    flywheelMotor.setIdleMode(IdleMode.kBrake);
  }
////double getPosition) 
  public void getSpinEncoder() {
    return flywheelMotor.getEncoder().getPosition();
  }


public void spinFlywheel(double speed){
  flywheelMotor.set(speed);
}
public void stopFlywheel(){
  flywheelMotor.set(0);
}
}

@Override
public void periodic() {
  // This method will be called once per scheduler run
}
