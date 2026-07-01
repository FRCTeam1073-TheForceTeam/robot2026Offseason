// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BallisticShot;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.TargetFinder;
import frc.robot.utilities.ShooterTable;

/**
 * Continuously update hood position based on automatically ranged shot calculation.
 */
public class TrackHood extends Command
{
  private final ShooterHood shooterHood;
  private final TargetFinder tf;
  private final ShooterTable st;
  private final BallisticShot bs;
  private final boolean useShooterTable;

  public TrackHood(ShooterHood shooterHood, TargetFinder tf, ShooterTable st, BallisticShot bs, boolean useShooterTable)
  {
    this.shooterHood = shooterHood;
    this.tf = tf;
    this.st = st;
    this.bs = bs;
    this.useShooterTable = useShooterTable;

    addRequirements(shooterHood);
  }

  public TrackHood(ShooterHood shooterHood, TargetFinder tf, ShooterTable st, BallisticShot bs)
  {
    this(shooterHood, tf, st, bs, false);
  }

  @Override
  public void initialize()
  {
  }

  @Override
  public void execute()
  {
    double range = tf.getRangeToTargetMeters();
    if (useShooterTable) {
      double targetAngle = st.getHoodAngle(range);
      shooterHood.setPosition(targetAngle);
    } else {
      BallisticShot.Shot shot = bs.getShot();
      shooterHood.setPosition(shot.hoodAngle);
    }
  }

  @Override
  public void end(boolean interrupted)
  {
    shooterHood.stop();
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }
}
