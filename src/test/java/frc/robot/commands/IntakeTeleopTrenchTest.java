package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.commands.TrenchTestDoubles.FakeOI;
import frc.robot.commands.TrenchTestDoubles.FakeZoneFinder;
import frc.robot.commands.TrenchTestDoubles.RecordingIntake;
import frc.robot.subsystems.Intake;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests the TRENCH override in IntakeTeleop: when localization places the robot in a trench, the intake must deploy.
 *
 * The blocker is stowed in these tests, so the trench rule applies in full. The alliance-zone rules are covered in
 * IntakeTeleopAllianceZoneTest.
 */
public class IntakeTeleopTrenchTest {
    private RecordingIntake intake;
    private FakeOI oi;
    private FakeZoneFinder zone;
    private IntakeTeleop cmd;

    @BeforeEach
    public void setUp() {
        intake = new RecordingIntake();
        oi = new FakeOI();
        zone = new FakeZoneFinder();
        cmd = new IntakeTeleop(intake, oi, zone, new DumperBlockerTestDoubles.RecordingDumperBlocker());
    }

    @Test
    public void testTrenchDeploysIntakeWhenStowed() {
        intake.position = Intake.stowedPositionRadians;
        zone.zones.add("TRENCH");

        cmd.execute();

        assertEquals(Intake.deployedPositionRadians, intake.lastSetPosition, 1e-9,
            "Intake should be commanded out when the robot is in the trench");
    }

    @Test
    public void testTrenchDeploysIntakeFromMidTravel() {
        intake.position = -1.0;
        zone.zones.add("TRENCH");

        cmd.execute();

        assertEquals(Intake.deployedPositionRadians, intake.lastSetPosition, 1e-9,
            "Intake should keep deploying from mid-travel when the robot is in the trench");
    }

    @Test
    public void testTrenchStopsIntakeOnceDeployed() {
        intake.position = Intake.deployedPositionRadians;
        zone.zones.add("TRENCH");

        cmd.execute();

        assertTrue(intake.stopCalled,
            "Intake should stop moving once it is fully deployed while in the trench");
    }

    @Test
    public void testTrenchIgnoresDriverStowPress() {
        intake.position = Intake.stowedPositionRadians;
        zone.zones.add("TRENCH");

        cmd.execute();
        oi.driverRightBumper = true;
        cmd.execute();
        oi.driverRightBumper = false;
        cmd.execute();

        assertFalse(intake.commandedPositions.contains(Intake.stowedPositionRadians),
            "Intake should never be commanded to stow while the robot is in the trench");
    }

    @Test
    public void testOutsideTrenchStaysStowedByDefault() {
        intake.position = Intake.stowedPositionRadians;

        cmd.execute();

        assertEquals(Intake.stowedPositionRadians, intake.lastSetPosition, 1e-9,
            "Intake should remain stowed when no trench is detected");
    }

    @Test
    public void testNonTrenchZonesDoNotDeployIntake() {
        intake.position = Intake.stowedPositionRadians;
        zone.zones.add("NEUTRALZONE");
        zone.zones.add("BUMP");
        zone.zones.add("BLUEZONE");

        cmd.execute();

        assertEquals(Intake.stowedPositionRadians, intake.lastSetPosition, 1e-9,
            "Only the TRENCH zone should deploy the intake");
    }
}
