package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DumperBlocker;
import static frc.robot.subsystems.DumperBlocker.hardstopCurrent;

public class ZeroDumperBlocker extends Command 
{
    private final DumperBlocker dumperBlocker;

    /**
     * Creates a new ZeroDumperBlocker command.
     *
     * @param dumperBlocker The subsystem used by this command.
     * @param unsafe If unsafe, don't require the subsystem. Use only for auto prep.
     */

    public ZeroDumperBlocker(DumperBlocker dumperBlocker, boolean unsafe) 
    {
        this.dumperBlocker = dumperBlocker;

        if (!unsafe) {
            addRequirements(dumperBlocker);
        }
    }

    public ZeroDumperBlocker(DumperBlocker dumperBlocker)
    {
        this(dumperBlocker, false);
    }

    @Override
    public void initialize()
    {
        System.err.println("Zero Dumper Blocker");
    }

    @Override
    public void execute()
    {
        dumperBlocker.zero();
    }

    @Override
    public void end(boolean interrupted)
    {
        if (interrupted) {
            System.err.println("Zero Dumper Blocker Interupted");
        } else {
            System.err.println("Zero Dumper Blocker Finished");
        }
        dumperBlocker.setZero();
        dumperBlocker.stop();
    }

    @Override
    public boolean isFinished()
    {
        if (Math.abs(dumperBlocker.getTorqueCurrent()) > hardstopCurrent) {
            return true;
        }
        return false;
    }
}
