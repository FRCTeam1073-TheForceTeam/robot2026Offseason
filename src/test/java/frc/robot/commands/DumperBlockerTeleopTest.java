package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.commands.DumperBlockerTestDoubles.FakeIntake;
import frc.robot.commands.DumperBlockerTestDoubles.RecordingDumperBlocker;
import frc.robot.commands.TrenchTestDoubles.FakeOI;
import frc.robot.subsystems.DumperBlocker;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests DumperBlockerTeleop: the driver's A button toggles the blocker between stowed and deployed, but the blocker
 * only deploys while the intake is all the way in.
 */
public class DumperBlockerTeleopTest {
    private static final double TOLERANCE = 1e-9;

    private RecordingDumperBlocker blocker;
    private FakeIntake intake;
    private FakeOI oi;
    private DumperBlockerTeleop cmd;

    @BeforeEach
    public void setUp() {
        blocker = new RecordingDumperBlocker();
        intake = new FakeIntake();
        oi = new FakeOI();
        cmd = new DumperBlockerTeleop(blocker, intake, oi);
        cmd.initialize();
    }

    @Test
    public void testStowedWithoutButtonPress() {
        cmd.execute();

        assertEquals(DumperBlocker.stowedPosition, blocker.lastSetPosition, TOLERANCE,
            "Blocker should stay stowed when the driver has not pressed A");
    }

    @Test
    public void testAButtonDeploysBlockerWhenIntakeStowed() {
        intake.stowed = true;
        oi.driverAButton = true;

        cmd.execute();

        assertEquals(DumperBlocker.deployedPosition, blocker.lastSetPosition, TOLERANCE,
            "Pressing A should deploy the blocker when the intake is all the way in");
    }

    @Test
    public void testAButtonWaitsWhenIntakeOut() {
        intake.stowed = false;
        oi.driverAButton = true;

        cmd.execute();

        assertEquals(DumperBlocker.stowedPosition, blocker.lastSetPosition, TOLERANCE,
            "Blocker should not deploy while the intake is still out");
    }

    @Test
    public void testHoldingAButtonDoesNotRetoggle() {
        oi.driverAButton = true;
        cmd.execute();

        cmd.execute();
        cmd.execute();

        assertEquals(DumperBlocker.deployedPosition, blocker.lastSetPosition, TOLERANCE,
            "Holding A should not toggle the blocker back to stowed");
    }

    @Test
    public void testSecondPressStowsBlocker() {
        oi.driverAButton = true;
        cmd.execute();
        oi.driverAButton = false;
        cmd.execute();

        oi.driverAButton = true;
        cmd.execute();

        assertEquals(DumperBlocker.stowedPosition, blocker.lastSetPosition, TOLERANCE,
            "A second press of A should stow the blocker again");
    }

    @Test
    public void testIntakeLeavingRetractsBlockerWithoutPress() {
        oi.driverAButton = true;
        cmd.execute();
        oi.driverAButton = false;
        intake.stowed = false;

        cmd.execute();

        assertEquals(DumperBlocker.stowedPosition, blocker.lastSetPosition, TOLERANCE,
            "Blocker should retract as soon as the intake leaves the stowed position");
    }

    @Test
    public void testBlockerRedeploysWhenIntakeReturnsWithoutPress() {
        oi.driverAButton = true;
        cmd.execute();
        oi.driverAButton = false;
        intake.stowed = false;
        cmd.execute();

        intake.stowed = true;
        cmd.execute();

        assertEquals(DumperBlocker.deployedPosition, blocker.lastSetPosition, TOLERANCE,
            "Blocker should redeploy on its own once the intake is back in, without another press of A");
    }

    @Test
    public void testInitializeClearsExtendedToggle() {
        oi.driverAButton = true;
        cmd.execute();
        oi.driverAButton = false;

        cmd.initialize();
        cmd.execute();

        assertEquals(DumperBlocker.stowedPosition, blocker.lastSetPosition, TOLERANCE,
            "Initializing the command should clear the extended toggle");
    }

    @Test
    public void testEndStopsBlockerWhenInterrupted() {
        cmd.end(true);

        assertTrue(blocker.stopCalled, "Ending the command should stop the blocker");
    }

    @Test
    public void testEndStopsBlockerWhenFinished() {
        cmd.end(false);

        assertTrue(blocker.stopCalled, "Ending the command should stop the blocker");
    }

    @Test
    public void testIsNeverFinished() {
        assertFalse(cmd.isFinished(), "Teleop blocker command should run until it is replaced");
    }
}
