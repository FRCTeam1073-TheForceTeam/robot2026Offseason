// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Turret;

public final class WeekZeroAuto
{
  private WeekZeroAuto()
  {
  }

  public static Command create(Spindexer spindexer, Kicker kicker, Flywheel flywheel, ShooterHood shooterHood, Turret turret)
  {
    return Commands.sequence(
        Commands.print("Begining Sequence"),
        Commands.runOnce(() -> flywheel.setVelocity(14.0), flywheel),
        Commands.runOnce(() -> shooterHood.setPosition(1 * 0.1), shooterHood),
        Commands.runOnce(() -> turret.setPosition(0.0), turret),
        Commands.waitSeconds(3.0),
        Commands.runOnce(() -> kicker.setVelocity(4.5), kicker),
        Commands.runOnce(() -> spindexer.setVelocity(4.2), spindexer),
        Commands.waitSeconds(8.0),
        Commands.runOnce(() -> shooterHood.setPosition(0 * 0.1), shooterHood),
        Commands.runOnce(() -> spindexer.setVelocity(0.0), spindexer),
        Commands.runOnce(() -> kicker.setVelocity(0.0), kicker),
        Commands.runOnce(() -> flywheel.setVelocity(0.0), flywheel),
        Commands.print("Finished Sequence"));
  }
}
