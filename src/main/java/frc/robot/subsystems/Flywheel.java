// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.FlywheelConstants.*;

import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import static frc.robot.Constants.FlywheelConstants;
//import com.revrobotics.CANSparkFlex.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Flywheel extends SubsystemBase {
  /** Creates a new Flywheel. */
  
  private CANSparkFlex flywheelUpperMotor;
  
  private CANSparkFlex flywheelLowerMotor;

  private boolean flywheelMode;
 // private double setSpeed;
  //private double flywheelEncoder;


  public Flywheel() {
    flywheelUpperMotor = new CANSparkFlex(KFlywheelUpperMotor, MotorType.kBrushless);

    flywheelLowerMotor = new CANSparkFlex(KFlywheelLowerMotor, MotorType.kBrushless);
    
    // flywheelUpperMotor.setInverted(true);

  }

public void shootNote(){
  flywheelUpperMotor.set(KFlywheelSpeedUpper);
  flywheelLowerMotor.set(KFlywheelSpeedLower);
}

public void stopMotors(){
  flywheelUpperMotor.set(0);
  flywheelLowerMotor.set(0);
}
public void SpinFlywheel(double speed){
  flywheelUpperMotor.set(speed);
  flywheelLowerMotor.set(-speed);
}

public double getUpperMotorEncoder(){
  return flywheelUpperMotor.getEncoder().getVelocity();
}

public double getLowerMotorEncoder(){
  return flywheelLowerMotor.getEncoder().getVelocity();
}

public void spinUpperFlywheel(double speed){
  flywheelUpperMotor.set(speed);
}

public void spinLowerFlywheel(double speed){
  flywheelLowerMotor.set(speed);
}


@Override
public void periodic() {
  // This method will be called once per scheduler run
}


}
