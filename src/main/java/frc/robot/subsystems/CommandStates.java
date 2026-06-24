// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CommandStates extends SubsystemBase {
  /** Creates a new CommandStates. */
  private boolean isCollecting;
  private boolean isGlobalAligning;
  private boolean isLocalAligning;
  private boolean isLidarAligning;
  private boolean algae;

  public CommandStates() {
    isCollecting = false;

    isGlobalAligning = false;
    isLocalAligning = false;
    isLidarAligning = false;
  }

  public void setIsGlobalAligning(boolean val) {
    isGlobalAligning = val;
  }

  public boolean getIsGlobalAligning() {
    return isGlobalAligning;
  }

  public void setIsLocalAligning(boolean val) {
    isLocalAligning = val;
  }

  public boolean getIsLocalAligning() {
    return isLocalAligning;
  }

  public void setIsLidarAligning(boolean val) {
    isLidarAligning = val;
  }

  public boolean getIsLidarAligning() {
    return isLidarAligning;
  }

  public void setIsCollecting(boolean val) {
    isCollecting = val;
  }

  public boolean getIsCollecting() {
    return isCollecting;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("States/isCollecting", isCollecting);
    SmartDashboard.putBoolean("States/isGlobalAligning", isGlobalAligning);
    SmartDashboard.putBoolean("States/isLocalAligning", isLocalAligning);
    SmartDashboard.putBoolean("States/isLidarAligning", isLidarAligning);
  }
}
