
package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.commands.TrenchTestDoubles.FakeBallisticShot;
import frc.robot.commands.TrenchTestDoubles.FakeOI;
import frc.robot.commands.TrenchTestDoubles.FakeShooterTable;
import frc.robot.commands.TrenchTestDoubles.FakeTargetFinder;
import frc.robot.commands.TrenchTestDoubles.FakeZoneFinder;
import frc.robot.commands.TrenchTestDoubles.RecordingShooterHood;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.subsystems.ShooterHood;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests the TRENCH override in HoodTeleop: when the robot is in a trench the hood must be put back out of the way,
 * regardless of any shot the driver or operator requests.
 *
 * The exception is the trench inside our own alliance zone, where we shoot from, so the hood must stay active there.
 * Our alliance comes from the driver station, which these tests set through DriverStationSim.
 *
 * The own-zone exemption and the no-alliance rule are both covered here, along with the non-trench shot angles.
 */
public class HoodTeleopTrenchTest {
    private static final double TOLERANCE = 1e-9;

    private RecordingShooterHood hood;
    private FakeOI oi;
    private FakeTargetFinder tf;
    private FakeShooterTable table;
    private FakeZoneFinder zone;
    private FakeBallisticShot ballistic;
    private HoodTeleop cmd;

    @BeforeEach
    public void setUp() {
        hood = new RecordingShooterHood();
        oi = new FakeOI();
        tf = new FakeTargetFinder();
        table = new FakeShooterTable();
        zone = new FakeZoneFinder();
        ballistic = new FakeBallisticShot(tf);
        cmd = new HoodTeleop(hood, oi, tf, table, zone, ballistic);
    }

    @AfterEach
    public void tearDown() {
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
    }

    private static void setAlliance(AllianceStationID station, Alliance expected) {
        DriverStationSim.setAllianceStationId(station);
        DriverStationSim.notifyNewData();
        // Guard: the alliance tests are only meaningful if the driver station actually reports this alliance.
        assertEquals(expected, DriverStation.getAlliance().orElse(null),
            "DriverStationSim should make DriverStation report the alliance under test");
    }

    private static void clearAlliance() {
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
        // Guard: the no-alliance tests are only meaningful if the driver station reports no alliance.
        assertTrue(DriverStation.getAlliance().isEmpty(),
            "DriverStationSim should make DriverStation report no alliance");
    }

    @Test
    public void testTrenchRetractsHoodWhileShootingWithLookupTable() {
        table.hoodAngle = 0.6;
        zone.zones.add("TRENCH");
        oi.driverXButton = true;

        cmd.execute();

        assertEquals(ShooterHood.maxPositionRadians, hood.lastSetPosition, TOLERANCE,
            "Hood should be retracted in the trench instead of using the lookup-table angle");
    }

    @Test
    public void testTrenchRetractsHoodWhileShootingWithBallisticShot() {
        ballistic.hoodAngle = 0.9;
        zone.zones.add("TRENCH");
        oi.driverXButton = true;
        oi.ballisticShotMode = true;

        cmd.execute();

        assertEquals(ShooterHood.maxPositionRadians, hood.lastSetPosition, TOLERANCE,
            "Hood should be retracted in the trench instead of using the ballistic angle");
    }

    @Test
    public void testTrenchRetractsHoodWhilePassing() {
        tf.passing = true;
        zone.zones.add("TRENCH");
        oi.driverXButton = true;

        cmd.execute();

        assertEquals(ShooterHood.maxPositionRadians, hood.lastSetPosition, TOLERANCE,
            "Hood should be retracted in the trench instead of using the passing angle");
    }

    @Test
    public void testTrenchRetractsHoodWhenShootTriggerPulled() {
        table.hoodAngle = 0.6;
        zone.zones.add("TRENCH");
        oi.operatorLeftTrigger = 1.0;

        cmd.execute();

        assertEquals(ShooterHood.maxPositionRadians, hood.lastSetPosition, TOLERANCE,
            "Hood should be retracted in the trench when the operator trigger is pulled");
    }

    @Test
    public void testTrenchRetractsHoodForCornerShot() {
        zone.zones.add("TRENCH");
        oi.operatorYButton = true;

        cmd.execute();

        assertEquals(ShooterHood.maxPositionRadians, hood.lastSetPosition, TOLERANCE,
            "Hood should be retracted in the trench instead of taking the corner-shot angle");
    }

    @Test
    public void testTrenchRetractsHoodForTowerShot() {
        zone.zones.add("TRENCH");
        oi.operatorXButton = true;

        cmd.execute();

        assertEquals(ShooterHood.maxPositionRadians, hood.lastSetPosition, TOLERANCE,
            "Hood should be retracted in the trench instead of taking the tower-shot angle");
    }

    @Test
    public void testHoodNotRetractedInOwnBlueAllianceTrench() {
        setAlliance(AllianceStationID.Blue1, Alliance.Blue);
        table.hoodAngle = 0.6;
        zone.zones.add("BLUEZONE");
        zone.zones.add("TRENCH");
        oi.driverXButton = true;

        cmd.execute();

        assertEquals(0.6, hood.lastSetPosition, TOLERANCE,
            "Hood should stay active so a blue robot can shoot from the trench inside its own blue alliance zone");
    }

    @Test
    public void testHoodRetractsInOpponentRedAllianceTrenchWhenBlue() {
        setAlliance(AllianceStationID.Blue1, Alliance.Blue);
        table.hoodAngle = 0.6;
        zone.zones.add("REDZONE");
        zone.zones.add("TRENCH");
        oi.driverXButton = true;

        cmd.execute();

        assertEquals(ShooterHood.maxPositionRadians, hood.lastSetPosition, TOLERANCE,
            "Hood should retract when a blue robot is in the trench inside the opponent's red alliance zone");
    }

    @Test
    public void testHoodNotRetractedInOwnRedAllianceTrench() {
        setAlliance(AllianceStationID.Red1, Alliance.Red);
        table.hoodAngle = 0.6;
        zone.zones.add("REDZONE");
        zone.zones.add("TRENCH");
        oi.driverXButton = true;

        cmd.execute();

        assertEquals(0.6, hood.lastSetPosition, TOLERANCE,
            "Hood should stay active so a red robot can shoot from the trench inside its own red alliance zone");
    }

    @Test
    public void testHoodRetractsInOpponentBlueAllianceTrenchWhenRed() {
        setAlliance(AllianceStationID.Red1, Alliance.Red);
        table.hoodAngle = 0.6;
        zone.zones.add("BLUEZONE");
        zone.zones.add("TRENCH");
        oi.driverXButton = true;

        cmd.execute();

        assertEquals(ShooterHood.maxPositionRadians, hood.lastSetPosition, TOLERANCE,
            "Hood should retract when a red robot is in the trench inside the opponent's blue alliance zone");
    }

    @Test
    public void testHoodNotRetractedInBlueZoneTrenchWithNoAlliance() {
        clearAlliance();
        table.hoodAngle = 0.6;
        zone.zones.add("BLUEZONE");
        zone.zones.add("TRENCH");
        oi.driverXButton = true;

        cmd.execute();

        assertEquals(0.6, hood.lastSetPosition, TOLERANCE,
            "With no alliance selected the hood should not retract near the blue alliance zone trench");
    }

    @Test
    public void testHoodNotRetractedInRedZoneTrenchWithNoAlliance() {
        clearAlliance();
        table.hoodAngle = 0.6;
        zone.zones.add("REDZONE");
        zone.zones.add("TRENCH");
        oi.driverXButton = true;

        cmd.execute();

        assertEquals(0.6, hood.lastSetPosition, TOLERANCE,
            "With no alliance selected the hood should not retract near the red alliance zone trench");
    }

    @Test
    public void testOutsideTrenchShootsWithLookupTable() {
        table.hoodAngle = 0.6;
        oi.driverXButton = true;

        cmd.execute();

        assertEquals(0.6, hood.lastSetPosition, TOLERANCE,
            "Outside the trench the lookup-table angle should be used while shooting");
    }

    @Test
    public void testOutsideTrenchCornerShot() {
        oi.operatorYButton = true;

        cmd.execute();

        assertEquals(Math.toRadians(55.0), hood.lastSetPosition, TOLERANCE,
            "Outside the trench the corner-shot angle should apply");
    }
}
