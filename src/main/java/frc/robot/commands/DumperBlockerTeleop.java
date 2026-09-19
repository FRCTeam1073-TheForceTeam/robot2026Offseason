package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DumperBlocker;
import frc.robot.subsystems.OI;
import frc.robot.utilities.DashboardNames;

import static frc.robot.subsystems.DumperBlocker.hardstopCurrent;

public class DumperBlockerTeleop extends Command
{
    private final DumperBlocker dumperBlocker;
    private final OI oi;
    
    private boolean extended = false;
    private boolean lastAButton = false;

    public DumperBlockerTeleop(DumperBlocker dumperBlocker, OI oi)
    {
        this.dumperBlocker = dumperBlocker;
        this.oi = oi;

        addRequirements(dumperBlocker);
    }

    @Override
    public void initialize()
    {
        // dumperBlocker.stop();
        extended = false; 
    }

    @Override
    public void execute()
    {
        dumperBlocker.setVelocity(0);
        boolean aButton = oi.getDriverAButton();

        if (!lastAButton && aButton) {
            extended = !extended;
        } 

        lastAButton = aButton;

        if (extended) {
            dumperBlocker.setPosition(1.25);
        } else {
            dumperBlocker.setPosition(0.0);
        }

        SmartDashboard.putBoolean("DumperBlocker/extended", extended);

        // SmartDashboard.putBoolean(DashboardNames.DUMPER_BLOCKER_HARDSTOP.getKey(), atHardstop)
    }

    @Override
    public void end(boolean interrupted)
    {
        dumperBlocker.stop();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}

