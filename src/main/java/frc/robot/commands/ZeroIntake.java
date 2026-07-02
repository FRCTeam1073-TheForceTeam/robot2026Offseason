// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/**
 * Zero the intake by driving into the hard stop, seeing current spike and resetting position.
 */
public class ZeroIntake extends Command
{
  private final Intake intake;

  public ZeroIntake(Intake intake)
  {
    this.intake = intake;

    addRequirements(intake);
  }

  @Override
  public void initialize()
  {
  }

  @Override
  public void execute()
  {
    double velocity = -1.0;
    intake.setVelocity(velocity);
  }

  @Override
  public void end(boolean interrupted)
  {
    intake.zero();
    intake.stop();
  }

  @Override
  public boolean isFinished()
  {
    if (Math.abs(intake.getTorqueNm()) > 2.5) {
      return true;
    }
    return false;
  }
}
