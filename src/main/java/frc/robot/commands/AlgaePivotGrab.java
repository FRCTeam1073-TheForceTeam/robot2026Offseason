// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaePivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaePivotGrab extends Command {
  /** Creates a new AlgaePivotGrab. */
  AlgaePivot pivot;
  public AlgaePivotGrab(AlgaePivot pivot) {
    this.pivot = pivot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot.setRotatorVel(-10);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot.setRotatorPos(pivot.getRotatorPosition() - 0.5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivot.getRotatorLoad() > 5;
  }
}
