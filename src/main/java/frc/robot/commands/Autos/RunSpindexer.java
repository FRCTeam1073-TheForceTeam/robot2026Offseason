// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Spindexer;

/**
 * Run the spindexer until interrupted. Does not finish on its own.
 */
public class RunSpindexer extends Command
{
  private final Spindexer spindexer;

  private double targetVelocity;

  public RunSpindexer(Spindexer spindexer)
  {
    this.spindexer = spindexer;

    addRequirements(spindexer);
  }

  @Override
  public void initialize()
  {
    targetVelocity = 4.2;
  }

  @Override
  public void execute()
  {
    spindexer.setVelocity(targetVelocity);
  }

  @Override
  public void end(boolean interrupted)
  {
    spindexer.stop();
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }
}
