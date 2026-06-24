// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaePivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaePivotToPosition extends Command {
  /** Creates a new AlgaePivotToPosition. */
  AlgaePivot pivot;
  double targetPosition;
  boolean terminate;
  public 
  AlgaePivotToPosition(AlgaePivot pivot, double targetPosition, boolean terminate) {
    this.pivot = pivot;
    this.terminate = terminate;
    this.targetPosition = targetPosition;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(targetPosition == -1){
      targetPosition = pivot.getRotatorPosition() - 0.5;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(pivot.getRotatorPosition() - targetPosition) >= 0.5){
      pivot.setRotatorPos(targetPosition);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(terminate){
      return Math.abs(pivot.getRotatorPosition() - targetPosition) < 0.5;
    }
    else{
      return false;
    }
  }
}
