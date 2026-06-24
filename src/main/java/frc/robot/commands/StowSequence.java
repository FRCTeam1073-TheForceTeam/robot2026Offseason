// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralElevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StowSequence extends Command {
  public CoralElevator coralElevator;

  /** Creates a new StowSequence. */
  public StowSequence(CoralElevator coralElevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.coralElevator = coralElevator;
  }

  public Command create() {
    return new SequentialCommandGroup(
      new StowElevator(coralElevator),
      new ZeroElevator(coralElevator)
    );
  }
}
