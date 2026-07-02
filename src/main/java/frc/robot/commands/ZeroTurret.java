// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

/**
 * Zero the turret by driving into the hard stop, seeing the current spike and resetting position.
 */
public class ZeroTurret extends Command
{
  private static final double limitNm = 3.5;

  private final Turret turret;

  /**
   * Create a ZeroTurret Command
   *
   * @param turret the subsystem handle.
   * @param unsafe Don't require the subsystem. Don't use except in auto prep!
   */
  public ZeroTurret(Turret turret, boolean unsafe)
  {
    this.turret = turret;

    if (!unsafe) {
      addRequirements(turret);
    }
  }

  public ZeroTurret(Turret turret)
  {
    this(turret, false);
  }

  @Override
  public void initialize()
  {
    System.err.println("Zero Turret");
  }

  @Override
  public void execute()
  {
    double velocity = 2.0; // TODO: change this value
    turret.setVelocity(velocity);
  }

  @Override
  public void end(boolean interrupted)
  {
    if (interrupted) {
      System.err.println("ZeroTurret Interrupted!! ");
    } else {
      System.err.println("ZeroTurret Finished");
    }
    turret.zero();
    turret.stop();
  }

  @Override
  public boolean isFinished()
  {
    if (Math.abs(turret.getTorqueNm()) > limitNm) { // TODO: change limit
      return true;
    }
    return false;
  }
}
