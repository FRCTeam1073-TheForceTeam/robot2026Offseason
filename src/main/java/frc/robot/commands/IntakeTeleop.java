// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DumperBlocker;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.ZoneFinder;
import java.util.Set;

public class IntakeTeleop extends Command
{
  private final Intake intake;
  private final OI oi;
  private final ZoneFinder zone;
  private final DumperBlocker dumperBlocker;

  private boolean positionIn = true;
  private boolean lastBumperRight = false; // For click detect on button A.
  private boolean lastAButton = false;

  public IntakeTeleop(Intake intake, OI oi, ZoneFinder zone, DumperBlocker dumperBlocker)
  {
    this.intake = intake;
    this.oi = oi;
    this.zone = zone;
    this.dumperBlocker = dumperBlocker;

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
    boolean aButton = oi.getDriverAButton();
    if (!lastBumperRight && bumperRight) {
      // Toggle position:
      positionIn = !positionIn;
    }
    else if (!lastAButton && aButton) {
      positionIn = true;
    }

    lastBumperRight = bumperRight; // Keep track of button for toggle.
    lastAButton = aButton;

    // The intake and the dumper blocker want the same space, so the intake may only come out once the blocker is
    // stowed. The exception is the trench inside our own alliance zone, where we shoot from and the blocker may
    // stay out.
    Set<String> zones = zone.getZones();
    String ownZone = ZoneFinder.ownAllianceZone();
    boolean ownTrench = zones.contains("TRENCH") && ownZone != null && zones.contains(ownZone);
    boolean deployAllowed = dumperBlocker.isStowed() || ownTrench;

    if (!deployAllowed) {
      positionIn = true;
    } else if (zones.contains("TRENCH")) {
      // Trench: keep the intake out, so the driver can't stow it there.
      positionIn = false;
    }

    if (positionIn) {
      intake.setPosition(Intake.stowedPositionRadians);
    } else {
      if (Math.abs(intake.getPositionRadians()) <= 0.2) {
        intake.stop();
      } else {
        intake.setPosition(Intake.deployedPositionRadians);
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
