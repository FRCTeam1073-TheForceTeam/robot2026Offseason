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
    private boolean slowIn = false; // Bringing the intake in slowly (hopper full).
    private boolean lastBumperRight = false; // For click detect on button A.
    private boolean lastAButton = false;
    private boolean lastBButton = false;

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
        boolean aButton = oi.getDriverAButton();
        boolean bButton = oi.getDriverBButton();
        /*
        if (zone.getZones().contains("TRENCH")) {
            positionIn = false;
        }*/
        if (!lastBumperRight && bumperRight) {
            // Toggle position:
            positionIn = !positionIn;
            slowIn = false;
        }
        else if (!lastAButton && aButton) {
            positionIn = true;
            slowIn = false;
        }
        else if (!lastBButton && bButton) {
            // Slow stow for when the hopper is full. Still counts as "in" for the bumper toggle.
            positionIn = true;
            slowIn = true;
        }

        lastBumperRight = bumperRight; // Keep track of button for toggle.
        lastAButton = aButton;
        lastBButton = bButton;

        if (positionIn) {
            if (slowIn) {
                intake.setPositionSlow(Intake.stowedPositionRadians);
            } else {
                intake.setPosition(Intake.stowedPositionRadians);
            }
        } else {
            if (Math.abs(intake.getPositionRadians()) <= 0.4) {
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
