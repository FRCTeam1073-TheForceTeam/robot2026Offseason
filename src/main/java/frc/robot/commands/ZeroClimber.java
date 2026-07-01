// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/**
 * Zero climber command.
 */
public class ZeroClimber extends Command
{
  private final Climber climber;

  /**
   * Creates a new ZeroClimber command.
   *
   * @param climber The subsystem used by this command.
   * @param unsafe If unsafe, don't require the subsystem. Use only for auto prep.
   */
  public ZeroClimber(Climber climber, boolean unsafe)
  {
    this.climber = climber;

    if (!unsafe) {
      addRequirements(climber);
    }
  }

  public ZeroClimber(Climber climber)
  {
    this(climber, false);
  }

  @Override
  public void initialize()
  {
    System.err.println("Zero Climber");
  }

  @Override
  public void execute()
  {
    double velocity = -0.03;
    climber.setVelocity(velocity);
  }

  @Override
  public void end(boolean interrupted)
  {
    if (interrupted) {
      System.err.println("Zero Climber Interrupted");
    } else {
      System.err.println("Zero Climber Finished");
    }
    climber.zero();
    climber.stop();
  }

  @Override
  public boolean isFinished()
  {
    if (Math.abs(climber.getForce()) > 6.0) {
      return true;
    }
    return false;
  }
}
