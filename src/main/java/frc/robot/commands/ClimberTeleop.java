// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.ZoneFinder;

public class ClimberTeleop extends Command
{
  private final Climber climber;
  private final OI oi;
  private final ZoneFinder zone;

  private double currentPosition;
  private double commandedPosition;

  public ClimberTeleop(Climber climber, OI oi, ZoneFinder zone)
  {
    this.climber = climber;
    this.oi = oi;
    this.zone = zone;

    addRequirements(climber);
  }

  @Override
  public void initialize()
  {
    currentPosition = climber.getClimberPosition();
  }

  @Override
  public void execute()
  {
    currentPosition = climber.getClimberPosition();

    double input;
    if (oi.getDriverMenuButton()) {
      input = 1.0;
    } else if (oi.getDriverViewButton()) {
      input = -1.0;
    } else {
      input = 0.0;
    }

    if (oi.getOperatorViewButton()) {
      commandedPosition = 0.0582;
      currentPosition = commandedPosition;
    } else if (oi.getOperatorMenuButton()) {
      commandedPosition = 0.0;
      currentPosition = commandedPosition;
    } else {
      commandedPosition = currentPosition + (input * 0.1);
    }

    if (zone.getZones().contains("TRENCH")) {
      commandedPosition = 0.0;
    }

    climber.setPosition(commandedPosition);

    // 9 amps at bottom
    // 10 when lift (but have switch)
  }

  @Override
  public void end(boolean interrupted)
  {
    climber.stop();
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }
}
