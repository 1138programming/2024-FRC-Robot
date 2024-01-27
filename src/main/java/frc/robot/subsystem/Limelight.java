// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

// import edu.wpi.first.wpiutil.net.PortForwarder;

/** Add your docs here. */
public class Limelight extends SubsystemBase{

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry ty = table.getEntry("ty");
    private NetworkTableEntry ta = table.getEntry("ta");

    public Limelight(){
        tx = 0;
        ty = 0
    }
    @Override
    public void periodic(){
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
    }

    public void LEDOn(){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode"). setNumber(3);
    }

    public void LEDOff() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); //(turns limelight off)
    }
    
    public void LEDBlink() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2); //(blinks limelight)
    }
}
