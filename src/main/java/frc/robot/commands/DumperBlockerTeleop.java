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
    private boolean atHardstop = false;

    public DumperBlockerTeleop(DumperBlocker dumperBlocker, OI oi)
    {
        this.dumperBlocker = dumperBlocker;
        this.oi = oi;

        addRequirements(dumperBlocker);
    }

    @Override
    public void initialize()
    {
        dumperBlocker.stop();
        atHardstop = false; 
    }

    @Override
    public void execute()
    {
        boolean aButton = oi.getDriverAButton();

        if (!lastAButton && aButton) {
            extended = !extended;
            atHardstop = false;
        }

        lastAButton = aButton;

        if (atHardstop) {
            dumperBlocker.stop();
            return;
        }

        if (extended) {
            dumperBlocker.extend();
        }
        else {
            dumperBlocker.retract();
        }

        if (Math.abs(dumperBlocker.getTorqueCurrent()) > hardstopCurrent) {
            dumperBlocker.stop();
            atHardstop = true;
        }

        SmartDashboard.putBoolean(DashboardNames.DUMPER_BLOCKER_HARDSTOP.getKey(), atHardstop);
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
