// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Flywheel.SpinFlywheelAndRotate;
import frc.robot.commands.Flywheel.SpinFlywheelAndTilt;
import frc.robot.commands.Indexer.IndexerSpin;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.ShooterTilt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonShoot extends SequentialCommandGroup {
  /** Creates a new AutonShoot. */
  public AutonShoot(Base base, Indexer indexer, Flywheel flywheel, ShooterTilt shooterTilt) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new WaitCommand(0.8),
        new SpinFlywheelAndRotate(flywheel, base, shooterTilt)
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(0.25), 
        new SpinFlywheelAndRotate(flywheel, base, shooterTilt),
        new IndexerSpin(indexer)
      )
    );
  }
}
