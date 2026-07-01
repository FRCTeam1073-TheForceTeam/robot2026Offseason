// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.OI;

public class KickerTeleop extends Command
{
  private final Kicker kicker;
  private final OI oi;

  public KickerTeleop(Kicker kicker, OI oi)
  {
    this.kicker = kicker;
    this.oi = oi;

    addRequirements(kicker);
  }

  @Override
  public void initialize()
  {
  }

  @Override
  public void execute()
  {
    boolean bButton = oi.getOperatorBButton();

    if (oi.getOperatorRightTrigger() >= 0.1) {
      kicker.setVelocity(Kicker.shotSpeed);
    } else if (bButton) {
      kicker.setVelocity(-1.65); // Jam clearing.
    } else {
      kicker.stop();
    }
  }

  @Override
  public void end(boolean interrupted)
  {
    kicker.stop();
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }
}
