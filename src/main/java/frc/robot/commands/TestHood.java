// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.ShooterHood;

/**
 * Hood test command for shot tuning.
 */
public class TestHood extends Command
{
  private static final int maxLevel = 32;
  private static final double scaleFactor = 0.015625;

  private final ShooterHood shooterHood;
  private final OI oi;

  private int level = 0;
  private boolean lastLeftBumper = false;
  private boolean lastRightBumper = false;

  public TestHood(ShooterHood shooterHood, OI oi)
  {
    this.shooterHood = shooterHood;
    this.oi = oi;

    addRequirements(shooterHood);
  }

  @Override
  public void initialize()
  {
    level = 0;
  }

  @Override
  public void execute()
  {
    boolean leftBumper = oi.getOperatorLeftBumper();
    boolean rightBumper = oi.getOperatorRightBumper();

    if (leftBumper && !lastLeftBumper) {
      ++level;
    }

    if (rightBumper && !lastRightBumper) {
      --level;
    }

    level = (int) MathUtil.clamp(level, 0, maxLevel);

    double position = ShooterHood.maxPositionRadians - level * scaleFactor;
    shooterHood.setPosition(position);
    SmartDashboard.putNumber("TestHood/level", level);
    SmartDashboard.putNumber("TestHood/position", position);

    lastLeftBumper = leftBumper;
    lastRightBumper = rightBumper;
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
