// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.BallisticShot;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.TargetFinder;
import frc.robot.subsystems.Turret;
import frc.robot.utilities.ShooterTable;

public final class Autos
{
  private Autos()
  {
  }

  // Fully track hub with turret, flywheel and hood for shooting, requires localization.
  public static Command trackHub(Turret turret, Flywheel flywheel, ShooterHood shooterHood, TargetFinder hf, ShooterTable st, BallisticShot bs)
  {
    return Commands.parallel(
        new TrackHood(shooterHood, hf, st, bs),
        new TrackFlywheel(flywheel, hf, st, bs),
        new TrackTurret(turret, hf));
  }

  // Shoots with tracking into hub, requires localization.
  public static Command basicAutoShot(Spindexer spindexer, Kicker kicker, Turret turret, Flywheel flywheel, ShooterHood shooterHood, TargetFinder hf, ShooterTable st, BallisticShot bs)
  {
    return Commands.parallel(
        new TrackHood(shooterHood, hf, st, bs),
        new TrackFlywheel(flywheel, hf, st, bs),
        new TrackTurret(turret, hf),
        Commands.sequence(Commands.waitSeconds(2.0),
            Commands.parallel(
                new RunSpindexer(spindexer),
                new RunKicker(kicker)))).withTimeout(10.0);
  }

  // Simple fixed shot auto directly in front of hub requires no localization.
  public static Command hubAuto(Spindexer spindexer, Kicker kicker, Turret turret, Flywheel flywheel, ShooterHood shooterHood)
  {
    return Commands.parallel(
        Commands.runOnce(() -> shooterHood.setPosition(Math.toRadians(69.2)), shooterHood),
        Commands.runOnce(() -> flywheel.setVelocity(8.0), flywheel),
        Commands.runOnce(() -> turret.setPosition(0.0), turret),
        Commands.sequence(Commands.waitSeconds(2.0),
            Commands.parallel(
                new RunSpindexer(spindexer),
                new RunKicker(kicker)))).withTimeout(10.0);
  }
}
