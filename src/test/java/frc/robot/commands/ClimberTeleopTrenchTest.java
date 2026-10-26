package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.commands.TrenchTestDoubles.FakeOI;
import frc.robot.commands.TrenchTestDoubles.FakeZoneFinder;
import frc.robot.commands.TrenchTestDoubles.RecordingClimber;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests the TRENCH override in ClimberTeleop: while the robot is in a trench the climber must be held at zero.
 *
 * The override is active in ClimberTeleop.execute(), so these tests are expected to PASS. They guard against
 * regressions while the intake and hood trench behavior is restored.
 */
public class ClimberTeleopTrenchTest {
    private static final double TOLERANCE = 1e-9;

    // Climber claims DIO port 0 in its constructor and the HAL never releases it, so build it once per JVM.
    private static RecordingClimber sharedClimber;

    private RecordingClimber climber;
    private FakeOI oi;
    private FakeZoneFinder zone;
    private ClimberTeleop cmd;

    @BeforeEach
    public void setUp() {
        if (sharedClimber == null) {
            sharedClimber = new RecordingClimber();
        }
        climber = sharedClimber;
        climber.reset();
        oi = new FakeOI();
        zone = new FakeZoneFinder();
        cmd = new ClimberTeleop(climber, oi, zone);
    }

    @Test
    public void testTrenchHoldsClimberAtZeroWhenDriverRaises() {
        climber.position = 0.03;
        zone.zones.add("TRENCH");
        oi.driverMenuButton = true;

        cmd.execute();

        assertEquals(0.0, climber.lastSetPosition, TOLERANCE,
            "Climber should be held at zero in the trench even when the driver raises it");
    }

    @Test
    public void testTrenchHoldsClimberAtZeroWhenIdle() {
        climber.position = 0.03;
        zone.zones.add("TRENCH");

        cmd.execute();

        assertEquals(0.0, climber.lastSetPosition, TOLERANCE,
            "Climber should be held at zero in the trench when idle");
    }

    @Test
    public void testTrenchOverridesOperatorExtendPreset() {
        climber.position = 0.0;
        zone.zones.add("TRENCH");
        oi.operatorViewButton = true;

        cmd.execute();

        assertEquals(0.0, climber.lastSetPosition, TOLERANCE,
            "Climber should stay at zero in the trench even when the operator selects the extend preset");
    }

    @Test
    public void testOutsideTrenchDriverRaisesClimber() {
        climber.position = 0.02;
        oi.driverMenuButton = true;

        cmd.execute();

        assertEquals(0.02 + 0.1, climber.lastSetPosition, TOLERANCE,
            "Outside the trench the driver should be able to raise the climber");
    }

    @Test
    public void testOutsideTrenchOperatorExtendPreset() {
        climber.position = 0.0;
        oi.operatorViewButton = true;

        cmd.execute();

        assertEquals(0.0582, climber.lastSetPosition, TOLERANCE,
            "Outside the trench the operator extend preset should apply");
    }
}
