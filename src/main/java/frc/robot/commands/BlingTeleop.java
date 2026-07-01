// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Bling;
import frc.robot.subsystems.OI;

public class BlingTeleop extends Command
{
  private static final double alpha = 0.025; // TODO: tune alpha

  private final Bling bling;
  private final OI oi;

  private double batteryVoltage = 0.0;

  public BlingTeleop(Bling bling, OI oi)
  {
    this.bling = bling;
    this.oi = oi;

    addRequirements(bling);
  }

  @Override
  public void initialize()
  {
  }

  @Override
  public void execute()
  {
    // Set LEDs on the CANdle itself for battery...
    SolidColor color = new SolidColor(0, 20);
    batteryVoltage = RobotController.getBatteryVoltage() * alpha + (1.0 - alpha) * batteryVoltage; // Filtered battery voltage.
    if (batteryVoltage >= 12.4) {
      color = color.withColor(new RGBWColor(0, 0, 127)); // Color is dimmer to conserve battery
    } else if (batteryVoltage >= 12.1) {
      color = color.withColor(new RGBWColor(127, 0, 82));
    } else {
      // Scaled voltage dimming as we die...
      double delta = (12.1 - batteryVoltage) / 5.0; // Minimum is about 7V so range is about 5V scale it from ~ (right at 12.1) to 1 (down at 7 almost dead)
      int redColor = (int) (127 - 100 * delta);
      color = color.withColor(new RGBWColor(redColor, 0, 0));
    }

    bling.setCommand(color);
  }

  @Override
  public void end(boolean interrupted)
  {
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }
}
