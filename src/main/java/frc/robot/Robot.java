// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import au.grapplerobotics.CanBridge;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utilities.DashboardNames;

import com.revrobotics.util.StatusLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

public class Robot extends LoggedRobot
{
  private boolean firstInit = true;
  private Command autonomousCommand;

  private RobotContainer container;

  private String gameData = "";
  private double shiftTime = 0.0;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Initialize URCL
    Logger.registerURCL(URCL.startExternal());
    StatusLogger.disableAutoLogging(); // Disable REVLib's built-in logging

    // Start AdvantageKit logger
    Logger.start();
  }
  @Override
  public void robotInit()
  {
    // Required to support laser-can debugging and configuration.
    CanBridge.runTCP();

    try {
      container = new RobotContainer(); // Actually create robot container here so we can capture errors for debugging.
      System.err.println("******* ROBOT CONTAINER CREATED ******** ");
    } catch (RuntimeException e) {
      System.err.println("CREATION OF ROBOT CONTAINER THREW AN EXCEPTION!: " + e.getMessage());
    }
  }

  @Override
  public void robotPeriodic()
  {
    try {
      CommandScheduler.getInstance().run();
    } catch (RuntimeException e) {
      System.err.println("SCHEDULER RUN THREW EXCEPTION!: " + e.getMessage());
    }

    if (gameData.isEmpty()) {
      gameData = DriverStation.getGameSpecificMessage();
    } else {
      switch (gameData.charAt(0)) {
        case 'B':
          SmartDashboard.putString(DashboardNames.AUTO_WINNERS.getKey(), "Blue");
          break;
        case 'R':
          SmartDashboard.putString("Auto Winners", "Red");
          break;
        default:
          SmartDashboard.putString("Auto Winners", "Somethin Went Wrong");
          break;
      }

      SmartDashboard.putBoolean(DashboardNames.HUB_ACTIVE.getKey(), isHubActive());
      container.setHubActive(isHubActive());

      int seconds = (int) shiftTime;
      SmartDashboard.putNumber(DashboardNames.DRIVE_SHIFT_TIME.getKey(), seconds);
    }
  }

  @Override
  public void disabledInit()
  {
    System.err.println("Disabled Init...");
    try {
      // Delegate to container function:
      if (firstInit) {
        container.disabledInit();
      }
    } catch (RuntimeException e) {
      System.err.println("CONTAINER DISABLEDINIT THREW EXCEPTION!: " + e.getMessage());
    }
  }

  @Override
  public void disabledPeriodic()
  {
    try {
      SmartDashboard.putBoolean(DashboardNames.AUTO_HAVE_TRAJECTORY.getKey(), container.getHaveTraj());
      SmartDashboard.putString(DashboardNames.AUTO_TRAJECTORY.getKey(), container.getAutoTraj());
      // Delegate to container function:
      if (!container.getSelectedAuto().equals(container.getAutoTraj())) {
        container.setHaveTraj(container.disabledPeriodic());
      }
    } catch (RuntimeException e) {
      System.err.println("CONTAINER DISABLEDPERIODIC THREW EXCEPTION!: " + e.getMessage());
    }
  }

  @Override
  public void autonomousInit()
  {
    System.err.println("Autonomous Init...");

    try {
      firstInit = false;
      autonomousCommand = container.getAutonomousCommand();

      if (autonomousCommand != null) {
        CommandScheduler.getInstance().schedule(autonomousCommand);
      } else {
        System.err.println("UNEXPLAINED MISSING AUTONOMOUS COMMAND!");
      }
    } catch (RuntimeException e) {
      System.err.println("AUTONOMOUS INIT THREW EXCEPTION!: " + e.getMessage());
    }
  }

  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    System.err.println("TeleopInit...");

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    container.teleopInit(); // Let container schedule things at start of teleop.
  }

  public boolean isHubActive()
  {
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return false;
    }

    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    if (!DriverStation.isTeleopEnabled()) {
      return false;
      // if we arent in teleop or auto at this point we can assume there is no hub
    }
    double matchTime = DriverStation.getMatchTime();

    boolean weInactiveFirst = false;
    if (!gameData.isEmpty()) {
      if (gameData.charAt(0) == 'R' && alliance.get() == Alliance.Blue) {
        weInactiveFirst = true;
      } else if (gameData.charAt(0) == 'B' && alliance.get() == Alliance.Red) {
        weInactiveFirst = true;
      }
    } else {
      // gamedata is invalid default to true
      return true;
    }

    // We know if we're inactive first:
    boolean shift1Active = !weInactiveFirst;

    if (matchTime > 130) {
      shiftTime = matchTime - 130;
      return true;
    } else if (matchTime > 105) {
      shiftTime = matchTime - 105;
      return shift1Active;
    } else if (matchTime > 80) {
      shiftTime = matchTime - 80;
      return !shift1Active;
    } else if (matchTime > 55) {
      shiftTime = matchTime - 55;
      return shift1Active;
    } else if (matchTime > 30) {
      shiftTime = matchTime - 30;
      return !shift1Active;
    } else {
      shiftTime = matchTime;
      return true;
    }
  }

  @Override
  public void teleopPeriodic()
  {
  }

  @Override
  public void testInit()
  {
    container.testInit(); // Initialize test mode.
  }

  @Override
  public void testPeriodic()
  {
  }

  @Override
  public void simulationInit()
  {
  }

  @Override
  public void simulationPeriodic()
  {
  }
}
