// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TargetFinder;
import frc.robot.subsystems.Turret;
import frc.robot.utilities.DashboardNames;

public class TrackTurret extends Command
{
  private final Turret turret;
  private final TargetFinder targetFinder;

  private double targetPosition = 0.0;
  private double position = 0.0; // zeroed position is touching the hard stop

  public TrackTurret(Turret turret, TargetFinder targetFinder)
  {
    this.turret = turret;
    this.targetFinder = targetFinder;

    addRequirements(turret);
  }

  @Override
  public void initialize()
  {
    System.err.println("TrackTurret Init");
  }

  @Override
  public void execute()
  {
    targetPosition = targetFinder.getTurretToTargetAngleRadians();
    turret.setPosition(targetPosition);

    SmartDashboard.putNumber(DashboardNames.TRACK_TURRET_POSITION.getKey(), position);
    SmartDashboard.putNumber(DashboardNames.TRACK_TURRET_TARGET_POSITION.getKey(), targetPosition);
  }

  @Override
  public void end(boolean interrupted)
  {
    if (interrupted) {
      System.err.println("TrackTurret: Interrupted!");
    }
    turret.stop();
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }
}
