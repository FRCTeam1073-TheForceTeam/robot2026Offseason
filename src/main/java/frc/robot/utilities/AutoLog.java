// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Timestamped console log of everything autonomous does, from autonomousInit until teleop or
 * disabled. Times are milliseconds since autonomousInit, so a delay before the robot moves shows
 * up as a gap between two lines.
 */
public final class AutoLog
{
  private static volatile boolean active = false;
  private static volatile double startSeconds = 0.0;
  private static volatile boolean motionLogged = false;

  private AutoLog()
  {
  }

  /** Call from autonomousInit, before the auto command is built. */
  public static void start()
  {
    startSeconds = Timer.getFPGATimestamp();
    motionLogged = false;
    active = true;
    event("autonomousInit");
  }

  /** Call when auto ends (teleop or disabled). Does nothing if auto was not running. */
  public static void stop(String reason)
  {
    if (active) {
      event("auto stopped: " + reason);
      active = false;
    }
  }

  /** Logs every command that starts, finishes or is interrupted while auto is running. */
  public static void attachScheduler()
  {
    CommandScheduler scheduler = CommandScheduler.getInstance();
    scheduler.onCommandInitialize(command -> event("init " + command.getName()));
    scheduler.onCommandFinish(command -> event("finish " + command.getName()));
    scheduler.onCommandInterrupt(command -> event("interrupt " + command.getName()));
  }

  public static double elapsedMs()
  {
    return (Timer.getFPGATimestamp() - startSeconds) * 1000.0;
  }

  public static void event(String message)
  {
    if (active) {
      System.err.printf("[AUTO +%.0f ms] %s%n", elapsedMs(), message);
    }
  }

  /** Logs the first time the drivetrain is commanded to move during this auto. */
  public static void motion(String message)
  {
    if (active && !motionLogged) {
      motionLogged = true;
      event("ROBOT MOVING: " + message);
    }
  }
}
