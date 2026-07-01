// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BallisticShot;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.TargetFinder;
import frc.robot.utilities.ShooterTable;

public class TrackFlywheel extends Command
{
  private final Flywheel flywheel;
  private final TargetFinder tf;
  private final ShooterTable st;
  private final BallisticShot bs;
  private final boolean lookupTable;

  public TrackFlywheel(Flywheel flywheel, TargetFinder tf, ShooterTable st, BallisticShot bs, boolean lookupTable)
  {
    this.flywheel = flywheel;
    this.tf = tf;
    this.st = st;
    this.bs = bs;
    this.lookupTable = lookupTable;

    addRequirements(flywheel);
  }

  public TrackFlywheel(Flywheel flywheel, TargetFinder tf, ShooterTable st, BallisticShot bs)
  {
    this(flywheel, tf, st, bs, false);
  }

  @Override
  public void initialize()
  {
  }

  @Override
  public void execute()
  {
    double range = tf.getRangeToTargetMeters();

    if (lookupTable) {
      double targetSpeed = st.getFlywheelVelocity(range);
      flywheel.setVelocity(targetSpeed);
    } else {
      BallisticShot.Shot shot = bs.getShot();
      flywheel.setVelocity(shot.flywheelSpeed);
    }
  }

  @Override
  public void end(boolean interrupted)
  {
    flywheel.stop();
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }
}
