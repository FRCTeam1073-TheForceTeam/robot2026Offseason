// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.OI;
import frc.robot.utilities.DashboardNames;

/**
 * Flywheel Test command for shot tuning.
 */
public class TestFlywheel extends Command
{
  private static final double scaleFactor = 0.20; // Speed per level.
  private static final int maxLevel = 120;

  private final Flywheel flywheel;
  private final OI oi;

  private boolean lastDPadUp = false;
  private boolean lastDPadDown = false;
  private int level = 0;

  public TestFlywheel(Flywheel flywheel, OI oi)
  {
    this.flywheel = flywheel;
    this.oi = oi;

    addRequirements(flywheel);
  }

  @Override
  public void initialize()
  {
    level = 0;
  }

  @Override
  public void execute()
  {
    boolean dPadUp = oi.getOperatorDPadUp();
    boolean dPadDown = oi.getOperatorDPadDown();

    if (dPadUp && !lastDPadUp) {
      ++level;
    }

    if (dPadDown && !lastDPadDown) {
      --level;
    }

    level = (int) MathUtil.clamp(level, 0, maxLevel);

    flywheel.setVelocity(level * scaleFactor);
    SmartDashboard.putNumber(DashboardNames.TEST_FLYWHEEL_LEVEL.getKey(), level);
    SmartDashboard.putNumber(DashboardNames.TEST_FLYWHEEL_SPEED.getKey(), level * scaleFactor);

    // Change detector:
    lastDPadDown = dPadDown;
    lastDPadUp = dPadUp;
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
