package frc.robot.commands;

import static frc.robot.commands.TrenchTestDoubles.clearAlliance;
import static frc.robot.commands.TrenchTestDoubles.setAlliance;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.commands.DumperBlockerTestDoubles.RecordingDumperBlocker;
import frc.robot.commands.TrenchTestDoubles.FakeOI;
import frc.robot.commands.TrenchTestDoubles.FakeZoneFinder;
import frc.robot.commands.TrenchTestDoubles.RecordingIntake;
import frc.robot.subsystems.DumperBlocker;
import frc.robot.subsystems.Intake;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests how IntakeTeleop uses the alliance zones to decide whether the intake may deploy in a trench. The dumper
 * blocker sits on the intake, so the blocker's position is what gates the intake when we are not in our own zone.
 *
 * Rules:
 * - Our own alliance zone's trench: the intake deploys whether or not the blocker is extended.
 * - The opponent's alliance zone's trench: the intake deploys only when the blocker is stowed.
 * - No alliance reported: both alliance zones count as the opponent's, so the blocker gates the intake in either.
 */
public class IntakeTeleopAllianceZoneTest {
    private RecordingIntake intake;
    private FakeOI oi;
    private FakeZoneFinder zone;
    private RecordingDumperBlocker blocker;
    private IntakeTeleop cmd;

    @BeforeEach
    public void setUp() {
        intake = new RecordingIntake();
        oi = new FakeOI();
        zone = new FakeZoneFinder();
        blocker = new RecordingDumperBlocker();
        cmd = new IntakeTeleop(intake, oi, zone, blocker);
    }

    @AfterEach
    public void tearDown() {
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
    }

    // Blue alliance: BLUEZONE is ours, REDZONE is the opponent's.

    @Test
    public void testBlueOwnZoneTrenchDeploysWhenBlockerStowed() {
        setAlliance(AllianceStationID.Blue1, Alliance.Blue);
        assertTrenchDeploys("BLUEZONE", DumperBlocker.stowedPosition, true);
    }

    @Test
    public void testBlueOwnZoneTrenchDeploysWhenBlockerExtended() {
        setAlliance(AllianceStationID.Blue1, Alliance.Blue);
        assertTrenchDeploys("BLUEZONE", DumperBlocker.deployedPosition, true);
    }

    @Test
    public void testBlueOpponentZoneTrenchDeploysWhenBlockerStowed() {
        setAlliance(AllianceStationID.Blue1, Alliance.Blue);
        assertTrenchDeploys("REDZONE", DumperBlocker.stowedPosition, true);
    }

    @Test
    public void testBlueOpponentZoneTrenchStaysStowedWhenBlockerExtended() {
        setAlliance(AllianceStationID.Blue1, Alliance.Blue);
        assertTrenchDeploys("REDZONE", DumperBlocker.deployedPosition, false);
    }

    // Red alliance: REDZONE is ours, BLUEZONE is the opponent's.

    @Test
    public void testRedOwnZoneTrenchDeploysWhenBlockerStowed() {
        setAlliance(AllianceStationID.Red1, Alliance.Red);
        assertTrenchDeploys("REDZONE", DumperBlocker.stowedPosition, true);
    }

    @Test
    public void testRedOwnZoneTrenchDeploysWhenBlockerExtended() {
        setAlliance(AllianceStationID.Red1, Alliance.Red);
        assertTrenchDeploys("REDZONE", DumperBlocker.deployedPosition, true);
    }

    @Test
    public void testRedOpponentZoneTrenchDeploysWhenBlockerStowed() {
        setAlliance(AllianceStationID.Red1, Alliance.Red);
        assertTrenchDeploys("BLUEZONE", DumperBlocker.stowedPosition, true);
    }

    @Test
    public void testRedOpponentZoneTrenchStaysStowedWhenBlockerExtended() {
        setAlliance(AllianceStationID.Red1, Alliance.Red);
        assertTrenchDeploys("BLUEZONE", DumperBlocker.deployedPosition, false);
    }

    // No alliance: both zones count as the opponent's, so the blocker gates the intake in either one.

    @Test
    public void testNoAllianceBlueZoneTrenchDeploysWhenBlockerStowed() {
        clearAlliance();
        assertTrenchDeploys("BLUEZONE", DumperBlocker.stowedPosition, true);
    }

    @Test
    public void testNoAllianceBlueZoneTrenchStaysStowedWhenBlockerExtended() {
        clearAlliance();
        assertTrenchDeploys("BLUEZONE", DumperBlocker.deployedPosition, false);
    }

    @Test
    public void testNoAllianceRedZoneTrenchDeploysWhenBlockerStowed() {
        clearAlliance();
        assertTrenchDeploys("REDZONE", DumperBlocker.stowedPosition, true);
    }

    @Test
    public void testNoAllianceRedZoneTrenchStaysStowedWhenBlockerExtended() {
        clearAlliance();
        assertTrenchDeploys("REDZONE", DumperBlocker.deployedPosition, false);
    }

    // Trench with no alliance zone named: the blocker alone decides.

    @Test
    public void testBareTrenchDeploysWhenBlockerStowed() {
        clearAlliance();
        assertTrenchDeploys(null, DumperBlocker.stowedPosition, true);
    }

    @Test
    public void testBareTrenchStaysStowedWhenBlockerExtended() {
        clearAlliance();
        assertTrenchDeploys(null, DumperBlocker.deployedPosition, false);
    }

    // Outside the trench the zone doesn't matter; the blocker still keeps the intake in.

    @Test
    public void testOutsideTrenchDriverCannotDeployWhileBlockerExtended() {
        setAlliance(AllianceStationID.Blue1, Alliance.Blue);
        blocker.position = DumperBlocker.deployedPosition;
        zone.zones.add("BLUEZONE");
        intake.position = Intake.stowedPositionRadians;
        oi.driverRightBumper = true;

        cmd.execute();

        assertEquals(Intake.stowedPositionRadians, intake.lastSetPosition, 1e-9,
            "Driver should not be able to deploy the intake while the blocker is extended outside the trench");
    }

    @Test
    public void testOutsideTrenchDriverCanDeployWhenBlockerStowed() {
        setAlliance(AllianceStationID.Blue1, Alliance.Blue);
        blocker.position = DumperBlocker.stowedPosition;
        zone.zones.add("BLUEZONE");
        intake.position = Intake.stowedPositionRadians;
        oi.driverRightBumper = true;

        cmd.execute();

        assertEquals(Intake.deployedPositionRadians, intake.lastSetPosition, 1e-9,
            "Driver should be able to deploy the intake outside the trench when the blocker is stowed");
    }

    /**
     * Runs one loop of IntakeTeleop in a trench in the given zone (or with no zone named, if null) and checks whether
     * the intake was commanded out. Starts with the intake stowed and no button pressed.
     */
    private void assertTrenchDeploys(String allianceZone, double blockerPosition, boolean expectDeploy) {
        blocker.position = blockerPosition;
        intake.position = Intake.stowedPositionRadians;
        if (allianceZone != null) {
            zone.zones.add(allianceZone);
        }
        zone.zones.add("TRENCH");

        cmd.execute();

        double expected = expectDeploy ? Intake.deployedPositionRadians : Intake.stowedPositionRadians;
        assertEquals(expected, intake.lastSetPosition, 1e-9,
            "Trench in " + allianceZone + " with blocker at " + blockerPosition + " should "
                + (expectDeploy ? "deploy" : "stay stowed"));
    }
}
