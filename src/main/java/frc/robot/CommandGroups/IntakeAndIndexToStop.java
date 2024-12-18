// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.Indexer.IndexerLoadNote;
import frc.robot.commands.Intake.IntakeSpinIn;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAndIndexToStop extends ParallelRaceGroup {
  /** Creates a new IntakeAndIndexToStop. */
  public IntakeAndIndexToStop(Intake intake, Indexer indexer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeSpinIn(intake),
      new IndexerLoadNote(indexer)
    );
  }
}
