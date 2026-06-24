// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralElevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DetectElevatorHeight extends Command 
{
  CoralElevator elevator;
  double height;
  double tolerance;

  /** Creates a new DetectElevatorHeight. */
  public DetectElevatorHeight(CoralElevator elevator, int branchLevel, double tolerance) 
  {
    this.elevator = elevator;
    if(branchLevel == 1){
      height = 0.45;
    }
    else if(branchLevel == 2){
      height = 0.72;
    }
    else if (branchLevel == 3){
      height = 1.11;
    }
    else if (branchLevel == 4){
      height = 1.73;
    }
    else if (branchLevel == 5) {
      height = 1.8;
    }
    else if (branchLevel == 6) {
      height = 0.95;
    }
    else if (branchLevel == 7) {
      height = 1.44;
    }
    this.tolerance = tolerance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(elevator.getMeters() - height) < tolerance;
  }
}
