// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Bling extends SubsystemBase
{
  public static final int CANdleId = 30; // temp number

  private final CANdle candle;

  private boolean hasCommand = false;
  private SolidColor command;

  public Bling()
  {
    setName("Bling");
    // Might not be roboRio figure out what it is if not
    candle = new CANdle(CANdleId, CANBus.roboRIO());
  }

  public void setCommand(SolidColor cmd)
  {
    hasCommand = true;
    command = cmd;
  }

  public void stop()
  {
    hasCommand = false;
  }

  @Override
  public void periodic()
  {
    if (hasCommand) {
      candle.setControl(command);
    } else {
      candle.setControl(new SolidColor(8, 20).withColor(new RGBWColor(255, 255, 255)));
    }
  }

  public void blingWhite()
  {
    setCommand(new SolidColor(8, 20).withColor(new RGBWColor(255, 255, 255)));
  }

  // TODO: change these commands to on and off
  public void blingPurple()
  {
    setCommand(new SolidColor(8, 20).withColor(new RGBWColor(147, 112, 219)));
  }
}
