// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import static frc.robot.Constants.*;

public class DoSomethingMotor extends SubsystemBase {
  private CANSparkMax NicoMotor1;
  /** Creates a new DoSomethingMotor. */
  public DoSomethingMotor() {
    NicoMotor1 = new CANSparkMax(KNicoMotor1ID, MotorType.kBrushless);

    NicoMotor1.setIdleMode(IdleMode.kCoast)
    NicoMotor1.setInverted((KNicoMotor1IDInverted));;
  } 

  public void spinNicoMotor1(double speed) {
    NicoMotor1.set(speed);
  }

  public void stopNicoMotor1() {
    NicoMotor1.set(speed:0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
