package frc.robot.commands;

import static frc.robot.commands.TrenchTestDoubles.clearAlliance;
import static frc.robot.commands.TrenchTestDoubles.setAlliance;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.commands.DumperBlockerTestDoubles.FakeIntake;
import frc.robot.commands.DumperBlockerTestDoubles.RecordingDumperBlocker;
import frc.robot.commands.TrenchTestDoubles.FakeOI;
import frc.robot.subsystems.DumperBlocker;
import frc.robot.subsystems.Intake;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests how the intake and the dumper blocker share the driver's A button and wait on each other.
 *
 * These tests run the intake and blocker commands together against the same OI and intake. The blocker waits for the
 * intake to be stowed; the intake's own A-button stow must still work while the blocker is extended.
 */
public class DumperBlockerIntakeInterlockTest {
    private FakeIntake intake;
    private FakeOI oi;
    private RecordingDumperBlocker blocker;
    private TrenchTestDoubles.FakeZoneFinder zone;
    private IntakeTeleop intakeCmd;
    private DumperBlockerTeleop blockerCmd;

    @BeforeEach
    public void setUp() {
        intake = new FakeIntake();
        oi = new FakeOI();
        blocker = new RecordingDumperBlocker();
        zone = new TrenchTestDoubles.FakeZoneFinder();
        intakeCmd = new IntakeTeleop(intake, oi, zone, blocker);
        blockerCmd = new DumperBlockerTeleop(blocker, intake, oi);
        blockerCmd.initialize();
    }

    /** Runs one loop of both commands, as the scheduler would. */
    private void runLoop() {
        intakeCmd.execute();
        blockerCmd.execute();
    }

    @Test
    public void testBlockerDeploysOnlyAfterIntakeStowed() {
        intake.stowed = true;
        oi.driverAButton = true;
        runLoop();

        assertEquals(DumperBlocker.deployedPosition, blocker.lastSetPosition, 1e-9,
            "Blocker should deploy when the driver presses A with the intake already stowed");
        assertEquals(Intake.stowedPositionRadians, intake.lastSetPosition, 1e-9,
            "Intake should stay stowed while the blocker is deployed");
    }

    @Test
    public void testAButtonWithIntakeOutStowsIntakeAndBlockerWaits() {
        intake.stowed = false;
        intake.position = Intake.deployedPositionRadians;
        oi.driverAButton = true;
        runLoop();

        assertEquals(Intake.stowedPositionRadians, intake.lastSetPosition, 1e-9,
            "Driver A should still stow the intake while the blocker is waiting on it");
        assertEquals(DumperBlocker.stowedPosition, blocker.lastSetPosition, 1e-9,
            "Blocker should wait while the intake is still out, even after A is pressed");
    }

    @Test
    public void testBlockerDeploysOnceIntakeFinishesStowing() {
        intake.stowed = false;
        intake.position = Intake.deployedPositionRadians;
        oi.driverAButton = true;
        runLoop();

        oi.driverAButton = false;
        intake.stowed = true;
        intake.position = Intake.stowedPositionRadians;
        runLoop();

        assertEquals(DumperBlocker.deployedPosition, blocker.lastSetPosition, 1e-9,
            "Blocker should deploy on its own once the intake has finished stowing");
    }

    @Test
    public void testIntakeDoesNotDeployWhileBlockerExtended() {
        // IntakeTeleop reads the blocker, so the driver's right bumper cannot deploy the intake while it is extended.
        blocker.position = DumperBlocker.deployedPosition;
        intake.stowed = true;
        oi.driverRightBumper = true;

        intakeCmd.execute();

        assertEquals(Intake.stowedPositionRadians, intake.lastSetPosition, 1e-9,
            "Intake should not deploy while the dumper blocker is extended");
    }

    @AfterEach
    public void tearDown() {
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
    }

    @Test
    public void testBlueRobotBlockerExtendedKeepsIntakeStowedInOpponentRedZoneTrench() {
        setAlliance(AllianceStationID.Blue1, Alliance.Blue);
        assertBlockerExtendedKeepsIntakeStowedInTrench("REDZONE");
    }

    @Test
    public void testRedRobotBlockerExtendedKeepsIntakeStowedInOpponentBlueZoneTrench() {
        setAlliance(AllianceStationID.Red1, Alliance.Red);
        assertBlockerExtendedKeepsIntakeStowedInTrench("BLUEZONE");
    }

    @Test
    public void testNoAllianceBlockerExtendedKeepsIntakeStowedInBlueZoneTrench() {
        clearAlliance();
        assertBlockerExtendedKeepsIntakeStowedInTrench("BLUEZONE");
    }

    @Test
    public void testNoAllianceBlockerExtendedKeepsIntakeStowedInRedZoneTrench() {
        clearAlliance();
        assertBlockerExtendedKeepsIntakeStowedInTrench("REDZONE");
    }

    @Test
    public void testBlueRobotBlockerStowedDeploysIntakeInOpponentRedZoneTrench() {
        setAlliance(AllianceStationID.Blue1, Alliance.Blue);
        assertTrenchDeploysIntake("REDZONE", DumperBlocker.stowedPosition);
    }

    @Test
    public void testRedRobotBlockerStowedDeploysIntakeInOpponentBlueZoneTrench() {
        setAlliance(AllianceStationID.Red1, Alliance.Red);
        assertTrenchDeploysIntake("BLUEZONE", DumperBlocker.stowedPosition);
    }

    @Test
    public void testNoAllianceBlockerStowedDeploysIntakeInBlueZoneTrench() {
        clearAlliance();
        assertTrenchDeploysIntake("BLUEZONE", DumperBlocker.stowedPosition);
    }

    @Test
    public void testBlueRobotBlockerExtendedDeploysIntakeInOwnBlueZoneTrench() {
        setAlliance(AllianceStationID.Blue1, Alliance.Blue);
        assertTrenchDeploysIntake("BLUEZONE", DumperBlocker.deployedPosition);
    }

    @Test
    public void testRedRobotBlockerExtendedDeploysIntakeInOwnRedZoneTrench() {
        setAlliance(AllianceStationID.Red1, Alliance.Red);
        assertTrenchDeploysIntake("REDZONE", DumperBlocker.deployedPosition);
    }

    /**
     * Intake stowed, robot in a trench inside the given alliance zone, no button pressed. Expected to deploy the intake.
     */
    private void assertTrenchDeploysIntake(String allianceZone, double blockerPosition) {
        blocker.position = blockerPosition;
        intake.stowed = true;
        zone.zones.add(allianceZone);
        zone.zones.add("TRENCH");

        intakeCmd.execute();

        assertEquals(Intake.deployedPositionRadians, intake.lastSetPosition, 1e-9,
            "Intake should deploy in the trench in " + allianceZone + " when the dumper blocker is at "
                + blockerPosition);
    }

    /** Extended blocker, intake stowed, robot in a trench inside the given alliance zone, no button pressed. */
    private void assertBlockerExtendedKeepsIntakeStowedInTrench(String allianceZone) {
        blocker.position = DumperBlocker.deployedPosition;
        intake.stowed = true;
        zone.zones.add(allianceZone);
        zone.zones.add("TRENCH");

        intakeCmd.execute();

        assertEquals(Intake.stowedPositionRadians, intake.lastSetPosition, 1e-9,
            "Trench rule should not deploy the intake while the dumper blocker is extended in " + allianceZone);
    }

    @Test
    public void testIntakeStaysStowedInTrenchWhileBlockerExtended() {
        // Guards the interlock: the trench rule must not deploy the intake while the blocker is extended.
        blocker.position = DumperBlocker.deployedPosition;
        intake.stowed = true;
        zone.zones.add("TRENCH");

        intakeCmd.execute();

        assertEquals(Intake.stowedPositionRadians, intake.lastSetPosition, 1e-9,
            "Intake should not deploy in the trench while the dumper blocker is extended");
    }

    @Test
    public void testZeroingRunsWhileIntakeOut() {
        // Zeroing drives the blocker toward stowed, so it is safe even while the intake is out.
        intake.stowed = false;
        intake.position = Intake.deployedPositionRadians;
        ZeroDumperBlocker zeroCmd = new ZeroDumperBlocker(blocker);
        zeroCmd.initialize();

        zeroCmd.execute();

        assertTrue(blocker.zeroCalled,
            "Operator zeroing should run even while the intake is out, since it only retracts the blocker");
    }

    @Test
    public void testBlockerRetractsInSameLoopIntakeLeavesStowed() {
        intake.stowed = true;
        oi.driverAButton = true;
        runLoop();
        oi.driverAButton = false;

        intake.stowed = false;
        intake.position = Intake.deployedPositionRadians;
        runLoop();

        assertEquals(DumperBlocker.stowedPosition, blocker.lastSetPosition, 1e-9,
            "Blocker should retract in the same loop the intake leaves the stowed position");
    }
}
