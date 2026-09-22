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
    private boolean deployLatched = false;

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
        deployLatched = false;
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

        // Deploying while the intake is still down would put the two into each other, so hold
        // the blocker stowed until the intake is all the way in. Once the blocker has been let
        // go it stays deployed, so the intake coming back out later does not retract it.
        if (extended) {
            if (deployLatched || intake.isStowed()) {
                deployLatched = true;
                dumperBlocker.setPosition(deployedPosition);
            } else {
                dumperBlocker.setPosition(stowedPosition);
            }
        } else {
            deployLatched = false;
            dumperBlocker.setPosition(stowedPosition);
        }

        SmartDashboard.putBoolean("DumperBlocker/extended", extended);
        SmartDashboard.putBoolean("DumperBlocker/WaitingForIntake", extended && !deployLatched);

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

