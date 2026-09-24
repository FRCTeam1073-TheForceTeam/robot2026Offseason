package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DumperBlocker;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OI;
import frc.robot.utilities.DashboardNames;

import static frc.robot.subsystems.DumperBlocker.deployedPosition;
import static frc.robot.subsystems.DumperBlocker.hardstopCurrent;
import static frc.robot.subsystems.DumperBlocker.stowedPosition;

public class DumperBlockerTeleop extends Command
{
    private final DumperBlocker dumperBlocker;
    private final Intake intake;
    private final OI oi;
    
    private boolean extended = false;
    private boolean lastAButton = false;

    /**
     * The intake is only read here, never commanded, so it is deliberately not a requirement -
     * adding it would fight IntakeTeleop for the subsystem.
     */
    public DumperBlockerTeleop(DumperBlocker dumperBlocker, Intake intake, OI oi)
    {
        this.dumperBlocker = dumperBlocker;
        this.intake = intake;
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

        // The blocker and a lowered intake want the same space, so the intake always wins:
        // the blocker only holds deployed while the intake is all the way in, and starts
        // retracting the same loop the intake leaves stowed - no latch, no waiting. The
        // extended toggle is left alone, so the blocker redeploys on its own once the intake
        // comes back in.
        boolean clearToDeploy = intake.isStowed();

        if (extended && clearToDeploy) {
            dumperBlocker.setPosition(deployedPosition);
        } else {
            dumperBlocker.setPosition(stowedPosition);
        }

        SmartDashboard.putBoolean("DumperBlocker/extended", extended);
        SmartDashboard.putBoolean("DumperBlocker/WaitingForIntake", extended && !clearToDeploy);

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

