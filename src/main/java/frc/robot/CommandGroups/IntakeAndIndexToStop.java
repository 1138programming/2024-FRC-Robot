// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Indexer.IndexerLoadNoteSlow;
import frc.robot.commands.Indexer.IndexerLoadNoteBack;
import frc.robot.commands.Indexer.IndexerLoadNoteFast;
import frc.robot.commands.Intake.IntakeSpinIn;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAndIndexToStop extends SequentialCommandGroup {
  /** Creates a new IntakeAndIndexToStop. */
  public IntakeAndIndexToStop(Intake intake, Indexer indexer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelRaceGroup(
        new IntakeSpinIn(intake),
        new IndexerLoadNoteFast(indexer)
      ),  
      new ParallelRaceGroup(
        new IntakeSpinIn(intake),
        new IndexerLoadNoteSlow(indexer)
      )
    );
  }
}
