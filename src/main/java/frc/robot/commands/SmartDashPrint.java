// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utilities.DashboardNames;

public class SmartDashPrint extends Command
{
  private final String s;

  public SmartDashPrint(String s)
  {
    this.s = s;
  }

  @Override
  public void initialize()
  {
    SmartDashboard.putString(DashboardNames.AUTO_EVENT.getKey(), s);
  }

  @Override
  public void execute()
  {
  }

  @Override
  public void end(boolean interrupted)
  {
  }

  @Override
  public boolean isFinished()
  {
    return true;
  }
}
