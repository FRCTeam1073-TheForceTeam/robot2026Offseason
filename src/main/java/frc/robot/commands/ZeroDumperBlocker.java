package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DumperBlocker;
import static frc.robot.subsystems.DumperBlocker.hardstopCurrent;

public class ZeroDumperBlocker extends Command 
{
    private final DumperBlocker dumperBlocker;

    public ZeroDumperBlocker(DumperBlocker dumperBlocker) 
    {
        this.dumperBlocker = dumperBlocker;

        addRequirements(dumperBlocker);
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
        // If it trigers too early add a hardstop counter.
        return false;
    }
}
