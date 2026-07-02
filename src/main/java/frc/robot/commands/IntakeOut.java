// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/**
 * Puts the intake out but can do it unsafely. Used for auto prep.
 */
public class IntakeOut extends Command
{
  private final Intake intake;

  /**
   * Create a IntakeOut Command
   *
   * @param intake the subsystem handle.
   * @param unsafe Don't require the subsystem. Don't use except in auto prep!
   */
  public IntakeOut(Intake intake, boolean unsafe)
  {
    this.intake = intake;

    if (!unsafe) {
      addRequirements(intake);
    }
  }

  public IntakeOut(Intake intake)
  {
    this(intake, false);
  }

  @Override
  public void initialize()
  {
    System.err.println("Intake Out");
  }

  @Override
  public void execute()
  {
    intake.setPosition(Math.toRadians(-0.1));
  }

  @Override
  public void end(boolean interrupted)
  {
    if (interrupted) {
      System.err.println("IntakeOut Interrupted!! ");
    } else {
      System.err.println("IntakeOut Finished");
    }
  }

  // Ends immediately:
  @Override
  public boolean isFinished()
  {
    return true;
  }
}
