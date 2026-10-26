package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.commands.DumperBlockerTestDoubles.RecordingDumperBlocker;
import frc.robot.subsystems.DumperBlocker;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests ZeroDumperBlocker: it drives the arm down at constant velocity, finishes once the arm has stalled on the
 * bottom hardstop, and only resets the encoder when it finishes on its own.
 *
 * Time is simulated with SimHooks so the startup and stall timers can be stepped without sleeping.
 */
public class ZeroDumperBlockerTest {
    private RecordingDumperBlocker blocker;
    private ZeroDumperBlocker cmd;

    @BeforeEach
    public void setUp() {
        SimHooks.pauseTiming();
        blocker = new RecordingDumperBlocker();
        cmd = new ZeroDumperBlocker(blocker);
        cmd.initialize();
    }

    @AfterEach
    public void tearDown() {
        SimHooks.resumeTiming();
    }

    @Test
    public void testExecuteDrivesDownAtZeroVelocity() {
        cmd.execute();

        assertTrue(blocker.zeroCalled, "Zeroing should drive the arm down with velocity control");
    }

    @Test
    public void testNotFinishedBeforeStartupElapses() {
        blocker.velocityRotorRps = 0.0;

        // Hold the stall for longer than the stall debounce, but stay inside the startup window.
        for (double t = 0.0; t < DumperBlocker.zeroStartupSeconds; t += 0.02) {
            assertFalse(cmd.isFinished(),
                "Zeroing should not finish before the arm has had time to start moving, even if it looks stalled");
            SimHooks.stepTiming(0.02);
        }
    }

    @Test
    public void testNotFinishedWhileArmStillMoving() {
        blocker.velocityRotorRps = DumperBlocker.zeroVelocity;
        SimHooks.stepTiming(DumperBlocker.zeroStartupSeconds + 1.0);

        assertFalse(cmd.isFinished(),
            "Zeroing should not finish while the arm is still moving down at full speed");
    }

    @Test
    public void testFinishesAfterStallHeld() {
        blocker.velocityRotorRps = 0.0;
        SimHooks.stepTiming(DumperBlocker.zeroStartupSeconds + 0.01);

        boolean finished = false;
        for (int i = 0; i < 20 && !finished; i++) {
            SimHooks.stepTiming(0.02);
            finished = cmd.isFinished();
        }

        assertTrue(finished, "Zeroing should finish once the arm has stalled on the hardstop long enough");
    }

    @Test
    public void testNotFinishedWhenVelocityAboveStallThreshold() {
        blocker.velocityRotorRps = DumperBlocker.zeroStallVelocity * 2.0;
        SimHooks.stepTiming(DumperBlocker.zeroStartupSeconds + 1.0);

        for (int i = 0; i < 20; i++) {
            SimHooks.stepTiming(0.02);
            assertFalse(cmd.isFinished(),
                "Zeroing should not finish while velocity stays above the stall threshold");
        }
    }

    @Test
    public void testEndWhenFinishedResetsEncoderAndStops() {
        cmd.end(false);

        assertTrue(blocker.stopCalled, "Zeroing should stop the blocker when it ends");
        assertTrue(blocker.setZeroCalled, "Zeroing should reset the encoder when it finishes on its own");
    }

    @Test
    public void testEndWhenInterruptedStopsWithoutResettingEncoder() {
        cmd.end(true);

        assertTrue(blocker.stopCalled, "Zeroing should stop the blocker when interrupted");
        assertFalse(blocker.setZeroCalled, "An interrupted zero must not reset the encoder, since the arm may not be on the hardstop");
    }
}
