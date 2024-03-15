// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Base.DriveForTimeX;
import frc.robot.commands.Base.DriveForTimeY;
import frc.robot.commands.Flywheel.SpinFlywheel;
import frc.robot.commands.Flywheel.StopFlywheel;
import frc.robot.commands.Flywheel.ThroughBore.SpinFlywheelSpeaker;
import frc.robot.commands.Indexer.IndexerSpin;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterTilt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SubBackPickShootAuton extends SequentialCommandGroup {
  /** Creates a new AimAndShoot. */
  public SubBackPickShootAuton(Base base, Flywheel flywheel, ShooterTilt shooterTilt, Indexer indexer, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new DriveForTimeY(base, 0, 3),
        new SpinFlywheelSpeaker(flywheel, shooterTilt)
        ),
      new ParallelDeadlineGroup(
        new DriveForTimeY(base, 0, 3),
        new SpinFlywheelSpeaker(flywheel, shooterTilt),
        new IndexerSpin(indexer)
        ),
        new ParallelDeadlineGroup(
          new DriveForTimeX(base, 0.3, 3),
          new StopFlywheel(flywheel),
          new IntakeAndIndex(intake, indexer)
      ),
      new ParallelDeadlineGroup(
        new DriveForTimeY(base, 0, 3),
        new SpinFlywheel(flywheel),
        new IndexerSpin(indexer)
      )
    ); 
  }
}
