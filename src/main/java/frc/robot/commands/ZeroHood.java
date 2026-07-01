// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterHood;

/**
 * Zero the hood by driving into hard stop, seeing current spike and resetting the zero position.
 */
public class ZeroHood extends Command
{
  private final ShooterHood shooterHood;

  public ZeroHood(ShooterHood shooterHood)
  {
    this.shooterHood = shooterHood;

    addRequirements(shooterHood);
  }

  @Override
  public void initialize()
  {
  }

  @Override
  public void execute()
  {
    double velocity = 0.2;
    shooterHood.setVelocity(velocity);
  }

  @Override
  public void end(boolean interrupted)
  {
    shooterHood.zero();
    shooterHood.stop();
  }

  @Override
  public boolean isFinished()
  {
    if (Math.abs(shooterHood.getTorqueNm()) > 2.2) {
      return true;
    }
    return false;
  }
}
