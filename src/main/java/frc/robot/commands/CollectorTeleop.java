// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;

public class CollectorTeleop extends Command
{
  private final Collector collector;
  private final OI oi;

  // Observes drivetrain, does not require it.
  private final Drivetrain dt;

  public CollectorTeleop(Collector collector, OI oi, Drivetrain dt)
  {
    this.collector = collector;
    this.oi = oi;
    this.dt = dt;

    // DO NOT REQUIRE DRIVETRAIN:
    addRequirements(collector);
  }

  @Override
  public void initialize()
  {
  }

  @Override
  public void execute()
  {
    // TODO: ASK SILLY DIRIVE TEAM!!
    if (Math.abs(oi.getDriverLeftTrigger()) >= 0.1) { // To eject fuel.
      collector.setVelocity(-4.0);
    } else if (Math.abs(oi.getDriverRightTrigger()) >= 0.1) {
      collector.setVelocity(9.14);
    } else {
      collector.stop();
    }
  }

  @Override
  public void end(boolean interrupted)
  {
    collector.stop(); // Default no-command state.
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }
}
