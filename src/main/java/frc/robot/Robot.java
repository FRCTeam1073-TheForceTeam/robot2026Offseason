// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import au.grapplerobotics.CanBridge;

public class Robot extends TimedRobot {
  private boolean firstInit = true;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    CanBridge.runTCP();
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    if (m_autonomousCommand != null){
      m_autonomousCommand.cancel();

    }
    if(firstInit){
      m_robotContainer.disabledInit();
    }
  }

  @Override
  public void disabledPeriodic() {
    if(!m_robotContainer.haveInitStartPos && firstInit) {
      m_robotContainer.haveInitStartPos = m_robotContainer.disabledPeriodic();
    }
    SmartDashboard.putBoolean("Have Initialized Start Pos", m_robotContainer.haveInitStartPos);
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    firstInit = false;
    m_robotContainer.autonomousInit();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
