package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DumperBlocker;
import static frc.robot.subsystems.DumperBlocker.zeroStallSeconds;
import static frc.robot.subsystems.DumperBlocker.zeroStallVelocity;
import static frc.robot.subsystems.DumperBlocker.zeroStartupSeconds;

/**
 * Zero the dumper blocker by driving down at a constant velocity until it stalls on the bottom
 * hardstop, then resetting position. This is the only command that uses velocity control.
 */
public class ZeroDumperBlocker extends Command
{
    private final DumperBlocker dumperBlocker;
    private final Timer timer = new Timer();
    private final Debouncer stallDebouncer = new Debouncer(zeroStallSeconds);

    public ZeroDumperBlocker(DumperBlocker dumperBlocker)
    {
        this.dumperBlocker = dumperBlocker;

        addRequirements(dumperBlocker);
    }

    @Override
    public void initialize()
    {
        System.err.println("Zero Dumper Blocker");
        timer.restart();
        stallDebouncer.calculate(false);
    }

    @Override
    public void execute()
    {
        dumperBlocker.zero();
    }

    @Override
    public void end(boolean interrupted)
    {
        dumperBlocker.stop();
        if (interrupted) {
            System.err.println("Zero Dumper Blocker Interupted");
        } else {
            System.err.println("Zero Dumper Blocker Finished");
            dumperBlocker.setZero();
        }
    }

    @Override
    public boolean isFinished()
    {
        // Velocity control holds the stall current low, so a current threshold may never trip.
        // Instead, once the arm has had time to get moving, finish when it stops moving.
        boolean stalled = timer.hasElapsed(zeroStartupSeconds)
            && Math.abs(dumperBlocker.getVelocityRotorRps()) < zeroStallVelocity;
        return stallDebouncer.calculate(stalled);
    }
}
