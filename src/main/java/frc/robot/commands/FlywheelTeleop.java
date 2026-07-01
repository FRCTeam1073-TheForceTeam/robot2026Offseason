// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BallisticShot;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.TargetFinder;
import frc.robot.utilities.ShooterTable;

public class FlywheelTeleop extends Command
{
  private final Flywheel flywheel;
  private final OI oi;
  private final TargetFinder tf;
  private final ShooterTable st;
  private final BallisticShot bs;

  public FlywheelTeleop(Flywheel flywheel, OI oi, TargetFinder tf, ShooterTable st, BallisticShot bs)
  {
    this.flywheel = flywheel;
    this.oi = oi;
    this.tf = tf;
    this.bs = bs;
    this.st = st;

    addRequirements(flywheel);
  }

  @Override
  public void initialize()
  {
    flywheel.setVelocity(0.0);
  }

  @Override
  public void execute()
  {
    if (Math.abs(oi.getOperatorLeftTrigger()) >= 0.1) {
      double rangeToTarget = tf.getRangeToTargetMeters();

      if (tf.isPassing()) {
        if (rangeToTarget < Units.inchesToMeters(270)) {
          flywheel.setVelocity(9.0);
        } else {
          flywheel.setVelocity(15.0);
        }
      } else if (oi.ballisticShotMode()) {
        // Use ballistic shot:
        BallisticShot.Shot shot = bs.getShot();
        flywheel.setVelocity(shot.flywheelSpeed);
      } else {
        // Using lookup table:
        double speed = st.getFlywheelVelocity(rangeToTarget);
        flywheel.setVelocity(speed);
      }
    } else if (oi.getOperatorYButton()) {
      flywheel.setVelocity(9.8); // Corner Shot
    } else if (oi.getOperatorXButton()) {
      flywheel.setVelocity(9.2); // Tower Shot
    } else {
      flywheel.stop();
    }
  }

  @Override
  public void end(boolean interrupted)
  {
    flywheel.stop(); // Coast/no-command.
  }

  @Override
  public boolean isFinished()
  {
    return false; // TODO: return true when finished
  }
}
