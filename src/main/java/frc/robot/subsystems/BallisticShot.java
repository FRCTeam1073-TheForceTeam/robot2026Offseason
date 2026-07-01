// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem continuously updates a cached value for a best-shot using compensated
 * ballistic trajectory.
 */
public class BallisticShot extends SubsystemBase
{
  public static class Shot
  {
    public final double flywheelSpeed;
    public final double hoodAngle;
    public final double shotTime;

    public Shot(double flywheelSpeed, double hoodAngle, double shotTime)
    {
      this.flywheelSpeed = flywheelSpeed;
      this.hoodAngle = hoodAngle;
      this.shotTime = shotTime;
    }
  }

  private static final double heightAboveHub = 1.0;
  private static final double hubHeight = 1.829;
  private static final double turretHeight = 0.40;
  private static final double efficiency = 0.80; // Calibration for transfer of flywheel velocity to fuel.
  private static final double hoodOffset = -0.16; // Calibration for shot exit vs. hood angle.
  private static final double gravity = 9.81;

  private final TargetFinder targetFinder;
  private Shot currentShot;

  public BallisticShot(TargetFinder targetFinder)
  {
    this.targetFinder = targetFinder;
    currentShot = new Shot(0.0, Math.toRadians(69.2), 2.0);
  }

  // Function to recompute shot:
  public static Shot computeShot(double range)
  {
    double tempHeightAboveHub = heightAboveHub;
    if (range > 3.0) {
      tempHeightAboveHub += (range - 3.0) * 0.5;
    }
    // hub is 6 feet tall or 1.829 meters
    double maxHeight = tempHeightAboveHub + hubHeight - turretHeight;
    double yVel = Math.sqrt(maxHeight * 2 * gravity);
    double timeToMaxHeight = yVel / gravity;
    double timeToFall = Math.sqrt(2 * tempHeightAboveHub / gravity);
    double shotTime = timeToMaxHeight + timeToFall;

    double xVel = range / shotTime;

    double hoodAngle = Math.atan2(yVel, xVel);

    double flywheelSpeed = Math.sqrt(xVel * xVel + yVel * yVel) / efficiency; // TODO: change efficiency

    hoodAngle += hoodOffset;

    // Special case for our mechanism limits:
    if (range < 1.5) {
      flywheelSpeed *= 0.95; // Downscale speed for very short shots.
    }

    return new Shot(flywheelSpeed, hoodAngle, shotTime);
  }

  @Override
  public void periodic()
  {
    double range = targetFinder.getRangeToTargetMeters();
    currentShot = computeShot(range); // Cache the computed shot value so we don't recompute it too often.
  }

  // Access the currently cached/latest shot value:
  public Shot getShot()
  {
    return currentShot;
  }
}
