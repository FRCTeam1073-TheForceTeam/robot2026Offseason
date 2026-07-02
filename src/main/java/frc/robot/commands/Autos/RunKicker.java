// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Kicker;

/**
 * Run the kicker until interrupted. Does not finish on its own.
 */
public class RunKicker extends Command
{
  private final Kicker kicker;

  private double targetVelocity;

  public RunKicker(Kicker kicker)
  {
    this.kicker = kicker;

    addRequirements(kicker);
  }

  @Override
  public void initialize()
  {
    targetVelocity = 4.5;
  }

  @Override
  public void execute()
  {
    kicker.setVelocity(targetVelocity);
  }

  @Override
  public void end(boolean interrupted)
  {
    kicker.stop();
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }
}
