// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Base.DriveForTimeY;
import frc.robot.commands.Flywheel.SpinFlywheelAmp;
import frc.robot.commands.Indexer.IndexerSpin;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.ShooterTilt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpAutoShoot extends SequentialCommandGroup {
  /** Creates a new AimAndShoot. */
  public AmpAutoShoot(Base base, Flywheel flywheel, ShooterTilt shooterTilt, Indexer indexer) {
    double speed = 0.63;
    SmartDashboard.putString("alliance", DriverStation.getAlliance().toString());
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new DriveForTimeY(base, speed, 1.2),
        new SpinFlywheelAmp(flywheel, shooterTilt)
      ),
      new ParallelDeadlineGroup(
        new DriveForTimeY(base, -speed, 0.7),
        new SpinFlywheelAmp(flywheel, shooterTilt)
      ),
      new ParallelDeadlineGroup(
        new DriveForTimeY(base, -speed, 0.45),
        new SpinFlywheelAmp(flywheel, shooterTilt),
        new IndexerSpin(indexer)
      )
    );
    
  }
}
