// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.Constants;

public class IndexerStop extends Command {
  private double speed;
  private Indexer indexer;


  /** Creates a new IndexerStop. */
  public IndexerStop(Indexer indexer) {
    this.indexer = indexer;
    this.speed = speed;
    addRequirements(indexer);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexer.indexerStop();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    indexer.indexerStop();

  }  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
