// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Flywheel.SpinFlywheel;
import frc.robot.commands.Flywheel.SpinUpFlywheel;
import frc.robot.commands.Indexer.IndexerSpin;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IndexAndShoot extends SequentialCommandGroup {
  /** Creates a new IndexAndShoot. */
  public IndexAndShoot(Flywheel flywheel, Indexer indexer) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new ParallelDeadlineGroup(
      new SpinUpFlywheel(flywheel)
      ),
    new ParallelDeadlineGroup(
      new WaitCommand(1.5),
      new SpinFlywheel(flywheel), 
      new IndexerSpin(indexer)
      )
    );
  }
}
