// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OI extends SubsystemBase
{
  private final XboxController driverController;
  private final XboxController operatorController;

  // TODO: make debouncers for individual buttons.
  private final Debouncer debouncer = new Debouncer(0.05, Debouncer.DebounceType.kBoth);

  private double leftXZero;
  private double leftYZero;
  private double rightXZero;
  private double rightYZero;

  private boolean hubActive = false;
  private boolean lastHubActive = false;

  // Top-level control of ballistic shot mode:
  private boolean ballisticShot = true;
  private boolean lastOperatorAButton = false;

  public OI()
  {
    setName("OI");
    driverController = new XboxController(0);
    operatorController = new XboxController(1);
  }

  @Override
  public void periodic()
  {
    boolean aButton = operatorController.getAButton();
    if (aButton && !lastOperatorAButton) {
      ballisticShot = !ballisticShot;
    }
    lastOperatorAButton = aButton;

    SmartDashboard.putBoolean("OI/BallisticShot", ballisticShot);
  }

  public boolean ballisticShotMode()
  {
    return ballisticShot;
  }

  public double getDriverLeftX()
  {
    return driverController.getLeftX();
  }

  public double getDriverLeftY()
  {
    return driverController.getLeftY();
  }

  public double getDriverRightX()
  {
    return -1 * driverController.getRightX();
  }

  public double getDriverRightY()
  {
    return driverController.getRightY();
  }

  public double getOperatorLeftX()
  {
    return -1 * operatorController.getLeftX();
  }

  public double getOperatorLeftY()
  {
    return operatorController.getLeftY();
  }

  public double getOperatorRightX()
  {
    return operatorController.getRightX();
  }

  public double getOperatorRightY()
  {
    return operatorController.getRightY();
  }

  public double getDriverLeftTrigger()
  {
    return driverController.getLeftTriggerAxis();
  }

  public double getDriverRightTrigger()
  {
    return driverController.getRightTriggerAxis();
  }

  public double getOperatorLeftTrigger()
  {
    return operatorController.getLeftTriggerAxis();
  }

  public double getOperatorRightTrigger()
  {
    return operatorController.getRightTriggerAxis();
  }

  public boolean getDriverAButton()
  {
    return driverController.getAButton();
  }

  public boolean getDriverBButton()
  {
    return driverController.getBButton();
  }

  public boolean getDriverXButton()
  {
    return driverController.getXButton();
  }

  public boolean getDriverYButton()
  {
    return driverController.getYButton();
  }

  public boolean getDriverMenuButton()
  {
    return driverController.getStartButton();
  }

  public boolean getDriverViewButton()
  {
    return driverController.getBackButton();
  }

  public boolean getDriverLeftBumper()
  {
    return driverController.getLeftBumperButton();
  }

  public boolean getDriverRightBumper()
  {
    return driverController.getRightBumperButton();
  }

  public int getDriverDPadAngle()
  {
    return driverController.getPOV();
  }

  public boolean getDriverDPadUp()
  {
    return driverController.getPOV() == 0;
  }

  public boolean getDriverDPadRight()
  {
    return driverController.getPOV() == 90;
  }

  public boolean getDriverDPadLeft()
  {
    return driverController.getPOV() == 270;
  }

  public boolean getDriverDPadDown()
  {
    return driverController.getPOV() == 180;
  }

  public void driverRumble()
  {
    driverController.setRumble(RumbleType.kBothRumble, 1.0);
  }

  public void driverStopRumble()
  {
    driverController.setRumble(RumbleType.kBothRumble, 0.0);
  }

  public boolean driverLeftStickPress()
  {
    return driverController.getLeftStickButton();
  }

  public boolean getOperatorAButton()
  {
    return operatorController.getAButton();
  }

  public boolean getOperatorBButton()
  {
    return operatorController.getBButton();
  }

  public boolean getOperatorXButton()
  {
    return operatorController.getXButton();
  }

  public boolean getOperatorYButton()
  {
    return operatorController.getYButton();
  }

  public boolean getOperatorMenuButton()
  {
    return operatorController.getStartButton();
  }

  public boolean getOperatorViewButton()
  {
    return operatorController.getBackButton();
  }

  public boolean getOperatorLeftBumper()
  {
    return operatorController.getLeftBumperButton();
  }

  public boolean getOperatorRightBumper()
  {
    return operatorController.getRightBumperButton();
  }

  public int getOperatorDPadAngle()
  {
    return operatorController.getPOV();
  }

  public boolean getOperatorDPadUp()
  {
    return operatorController.getPOV() == 0;
  }

  public boolean getOperatorDPadRight()
  {
    return operatorController.getPOV() == 90;
  }

  public boolean getOperatorDPadLeft()
  {
    return operatorController.getPOV() == 270;
  }

  public boolean getOperatorDPadDown()
  {
    return operatorController.getPOV() == 180;
  }

  public void operatorRumble()
  {
    operatorController.setRumble(RumbleType.kBothRumble, 1.0);
  }

  public void operatorStopRumble()
  {
    operatorController.setRumble(RumbleType.kBothRumble, 0.0);
  }

  public void zeroDriverController()
  {
    leftXZero = getDriverLeftX();
    leftYZero = getDriverLeftY();
    rightXZero = getDriverRightX();
    rightYZero = getDriverRightY();
  }

  public void zeroOperatorController()
  {
    leftXZero = getOperatorLeftX();
    leftYZero = getOperatorLeftY();
    rightXZero = getOperatorRightX();
    rightYZero = getOperatorRightY();
  }

  public void setHubActive(boolean active)
  {
    hubActive = active;
  }
}
