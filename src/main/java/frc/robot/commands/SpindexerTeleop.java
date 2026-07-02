// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.Spindexer;

public class SpindexerTeleop extends Command
{
  private final Spindexer spindexer;
  private final Kicker kicker;
  private final OI oi;

  public SpindexerTeleop(Spindexer spindexer, Kicker kicker, OI oi)
  {
    this.spindexer = spindexer;
    this.kicker = kicker;
    this.oi = oi;

    addRequirements(spindexer);
  }

  @Override
  public void initialize()
  {
  }

  @Override
  public void execute()
  {
    if (Math.abs(oi.getOperatorRightTrigger()) >= 0.1 && Math.abs(kicker.getVelocity()) >= 3.0) {
      spindexer.setVelocity(Spindexer.shotSpeed);
    } else if (oi.getOperatorBButton()) {
      spindexer.setVelocity(-2.0); // Jam clearing.
    } else {
      spindexer.stop();
    }
  }

  @Override
  public void end(boolean interrupted)
  {
    spindexer.stop(); // Default no-command state.
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }
}
