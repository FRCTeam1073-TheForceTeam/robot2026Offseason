// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BallisticShot;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.TargetFinder;
import frc.robot.subsystems.ZoneFinder;
import frc.robot.utilities.ShooterTable;

public class HoodTeleop extends Command
{
  private final ShooterHood shooterHood;
  private final OI oi;
  private final TargetFinder tf;
  private final ShooterTable st;
  private final ZoneFinder zone;
  private final BallisticShot bs;

  public HoodTeleop(ShooterHood shooterHood, OI oi, TargetFinder tf, ShooterTable st, ZoneFinder zone, BallisticShot bs)
  {
    this.shooterHood = shooterHood;
    this.oi = oi;
    this.tf = tf;
    this.st = st;
    this.bs = bs;
    this.zone = zone;

    addRequirements(shooterHood);
  }

  @Override
  public void initialize()
  {
  }

  @Override
  public void execute()
  {
    if (zone.getZones().contains("TRENCH")) {
      // Put the hood "back" to be out of the way.
      shooterHood.setPosition(ShooterHood.maxPositionRadians);
    } else if (Math.abs(oi.getOperatorLeftTrigger()) >= 0.1) {
      double rangeToTarget = tf.getRangeToTargetMeters();

      if (tf.isPassing()) {
        if (rangeToTarget < Units.inchesToMeters(330)) {
          shooterHood.setPosition(ShooterHood.minPositionRadians - Math.toRadians(10));
        } else if (rangeToTarget < Units.inchesToMeters(270)) {
          shooterHood.setPosition(ShooterHood.minPositionRadians);
        } else {
          shooterHood.setPosition(ShooterHood.minPositionRadians + Math.toRadians(13));
        }
      } else if (oi.ballisticShotMode()) {
        // Use ballistic shot:
        BallisticShot.Shot shot = bs.getShot();
        shooterHood.setPosition(shot.hoodAngle);
      } else {
        // Use lookup table:
        double angle = st.getHoodAngle(rangeToTarget);
        shooterHood.setPosition(angle);
      }
    } else if (oi.getOperatorYButton()) {
      // Corner shot:
      shooterHood.setPosition(Math.toRadians(55.00));
    } else if (oi.getOperatorXButton()) {
      // Tower shot:
      shooterHood.setPosition(Math.toRadians(61.00));
    } else {
      // Put the hood "back" to be out of the way.
      shooterHood.setPosition(ShooterHood.maxPositionRadians);
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
