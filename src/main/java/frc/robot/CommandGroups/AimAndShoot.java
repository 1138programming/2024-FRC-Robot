// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Base.RotateToSpeaker;
import frc.robot.commands.Flywheel.SpinFlywheelVelocity;
import frc.robot.commands.Flywheel.SpinUpFlywheel;
import frc.robot.commands.Indexer.IndexerSpin;
import frc.robot.commands.ShooterTilt.AutoAimShooterTilt;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.ShooterTilt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimAndShoot extends SequentialCommandGroup {
  /** Creates a new AimAndShoot. */
  public AimAndShoot(Base base, Flywheel flywheel, ShooterTilt shooterTilt, Indexer indexer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new AutoAimShooterTilt(shooterTilt),
        new RotateToSpeaker(base),
        new SpinUpFlywheel(flywheel)
      ),
      new ParallelRaceGroup(
        new IndexerSpin(indexer),
        new SpinFlywheelVelocity(flywheel)
      )
    );
  }
}
