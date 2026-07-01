// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TODO: finish the command; it is not complete yet

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.TargetFinder;
import frc.robot.subsystems.Turret;

public class TurretTeleop extends Command
{
  private final Turret turret;
  private final OI oi;
  private final TargetFinder targetFinder;
  private final Drivetrain drivetrain;

  private double targetAngle = 0.0;

  public TurretTeleop(Turret turret, OI oi, TargetFinder targetFinder, Drivetrain drivetrain)
  {
    this.turret = turret;
    this.oi = oi;
    this.targetFinder = targetFinder;
    this.drivetrain = drivetrain;

    addRequirements(turret);
  }

  @Override
  public void initialize()
  {
    System.err.println("TeleopTurret: Init");
  }

  @Override
  public void execute()
  {
    double leftX = oi.getOperatorLeftX();

    if (Math.abs(leftX) > 0.1) {
      targetAngle = Math.toRadians(330) * leftX;
    } else if (oi.getOperatorLeftTrigger() >= 0.1) {
      targetAngle = targetFinder.getTurretToTargetAngleRadians();
    } else {
      targetAngle = 0.0;
    }

    turret.setPosition(targetAngle);

    SmartDashboard.putNumber("TeleopTurret/targetAngle", targetAngle);
  }

  @Override
  public void end(boolean interrupted)
  {
    if (interrupted) {
      System.err.println("TeleopTurret: Interrupted!");
    }
    turret.stop();
  }

  @Override
  public boolean isFinished()
  {
    return false; // TODO: return true if it finishes
  }
}
