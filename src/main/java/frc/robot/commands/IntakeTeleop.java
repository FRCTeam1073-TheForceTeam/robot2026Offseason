// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.ZoneFinder;

public class IntakeTeleop extends Command
{
  private final Intake intake;
  private final OI oi;
  private final ZoneFinder zone;

  private boolean positionIn = true;
  private boolean lastBumperRight = false; // For click detect on button A.

  public IntakeTeleop(Intake intake, OI oi, ZoneFinder zone)
  {
    this.intake = intake;
    this.oi = oi;
    this.zone = zone;

    addRequirements(intake);
  }

  @Override
  public void initialize()
  {
  }

  @Override
  public void execute()
  {
    boolean bumperRight = oi.getDriverRightBumper();

    if (zone.getZones().contains("TRENCH")) {
      positionIn = false;
    } else if (!lastBumperRight && bumperRight) {
      // Toggle position:
      positionIn = !positionIn;
    }

    lastBumperRight = bumperRight; // Keep track of button for toggle.

    if (positionIn) {
      intake.setPosition(Math.toRadians(-122.0));
    } else {
      if (Math.abs(intake.getPositionRadians()) <= 0.2) {
        intake.stop();
      } else {
        intake.setPosition(Math.toRadians(-0.1));
      }
    }
  }

  @Override
  public void end(boolean interrupted)
  {
    intake.stop();
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }
}
