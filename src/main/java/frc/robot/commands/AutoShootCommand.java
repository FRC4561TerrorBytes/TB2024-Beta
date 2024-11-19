// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

public class AutoShootCommand extends Command {

 
  private final Shooter shooter;
  private final Indexer indexer;
  
  private final Drive drive;
  private double targetMPS;


  public AutoShootCommand( Shooter shooter, Indexer indexer,  Drive drive) {
   
    this.shooter = shooter;
    this.indexer = indexer;
    
    this.drive = drive;

    addRequirements(shooter, indexer, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Leds.getInstance().autoShootCommand = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   
    targetMPS = 25;

    shooter.setFlywheelSpeed(targetMPS);

    if (shooter.flywheelUpToSpeed(targetMPS)) {
      indexer.setIndexerSpeed(Constants.INDEXER_FEED_SPEED);
    
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopFlywheel();
    indexer.stopIndexer();
    
    
    Leds.getInstance().autoShootCommand = false;
    Logger.recordOutput("Auto Rotate/Rotating", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!indexer.noteInIndexer()) {
      for (int i = 0; i < 25; i++) {
        continue;
      }
      return true;
    } else {
      return false;
    }
  }
}
