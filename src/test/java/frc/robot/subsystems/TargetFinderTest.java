package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import java.util.Set;
import java.util.TreeSet;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests TargetFinder's choice between the hub (our alliance zone) and a pass (anywhere else), and its range output.
 *
 * With no alliance reported, ourZone stays null, so periodic() must treat every position as a pass rather than
 * looking up our zone.
 */
public class TargetFinderTest {
    private static final double TOLERANCE = 1e-9;

    private ZoneFinderTest.FakeLocalizer localizer;
    private FakeZoneFinder zoneFinder;
    private TargetFinder targetFinder;

    /** ZoneFinder double whose zones are set directly by each test. Uses a TreeSet, like the real ZoneFinder. */
    static class FakeZoneFinder extends ZoneFinder {
        final Set<String> zones = new TreeSet<>();

        FakeZoneFinder(Localizer localizer) {
            super(localizer);
        }

        @Override
        public Set<String> getZones() {
            return zones;
        }
    }

    @BeforeEach
    public void setUp() {
        localizer = new ZoneFinderTest.FakeLocalizer();
        zoneFinder = new FakeZoneFinder(localizer);
        targetFinder = new TargetFinder(localizer, zoneFinder);
        targetFinder.setBallisticShot(new BallisticShot(targetFinder));
    }

    @AfterEach
    public void tearDown() {
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
    }

    @Test
    public void testHubTargetedInOwnBlueAllianceZone() {
        setAlliance(AllianceStationID.Blue1, Alliance.Blue);
        zoneFinder.zones.add("BLUEZONE");

        targetFinder.periodic();

        assertFalse(targetFinder.isPassing(), "Robot in its own alliance zone should shoot at the hub, not pass");
        assertEquals(expectedRange(TargetFinder.BLUEHUB), targetFinder.getRangeToTargetMeters(), TOLERANCE,
            "Range should be measured to the blue hub");
    }

    @Test
    public void testHubTargetedInOwnRedAllianceZone() {
        setAlliance(AllianceStationID.Red1, Alliance.Red);
        zoneFinder.zones.add("REDZONE");

        targetFinder.periodic();

        assertFalse(targetFinder.isPassing(), "Robot in its own alliance zone should shoot at the hub, not pass");
        assertEquals(expectedRange(TargetFinder.REDHUB), targetFinder.getRangeToTargetMeters(), TOLERANCE,
            "Range should be measured to the red hub");
    }

    @Test
    public void testPassingOutsideOwnAllianceZone() {
        setAlliance(AllianceStationID.Blue1, Alliance.Blue);
        zoneFinder.zones.add("REDZONE");

        targetFinder.periodic();

        assertTrue(targetFinder.isPassing(), "Robot outside its own alliance zone should pass");
    }

    @Test
    public void testBluePassUsesRightHalfTargetWhenInRightHalf() {
        setAlliance(AllianceStationID.Blue1, Alliance.Blue);
        zoneFinder.zones.add("RIGHTHALF");

        targetFinder.periodic();

        assertTrue(targetFinder.isPassing());
        assertEquals(expectedRange(TargetFinder.BLUEPASS_R), targetFinder.getRangeToTargetMeters(), TOLERANCE,
            "Blue robot in the right half should pass to the right-side blue target");
    }

    @Test
    public void testPassWithNoAllianceFallsBackToFieldCenter() {
        // Alliance unknown: pass() never reads the zone set, so it falls back to the field center.
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();

        // periodic() has not run, so the turret pose is still the default pose rather than the offset turret.
        Pose2d target = targetFinder.pass();

        assertEquals(TargetFinder.FIELD_CENTER.relativeTo(new Pose2d()).getTranslation().getNorm(),
            target.getTranslation().getNorm(), TOLERANCE,
            "With no alliance reported, the pass target should be the field center");
    }

    @Test
    public void testPeriodicWithNoAllianceDoesNotThrow() {
        // ourZone is null until an alliance is seen, so getTargetPos() must not look it up in the zone set.
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
        zoneFinder.zones.add("BLUEZONE");

        assertDoesNotThrow(targetFinder::periodic,
            "periodic() should not throw when no alliance has been reported");
    }

    /** Range from the turret (robot pose at the origin plus the turret offset) to the given field target. */
    private static double expectedRange(Pose2d target) {
        Pose2d turret = new Pose2d().transformBy(TargetFinder.ROBOTOTURRET);
        return target.relativeTo(turret).getTranslation().getNorm();
    }

    private static void setAlliance(AllianceStationID station, Alliance expected) {
        DriverStationSim.setAllianceStationId(station);
        DriverStationSim.notifyNewData();
        assertEquals(expected, DriverStation.getAlliance().orElse(null),
            "DriverStationSim should make DriverStation report the alliance under test");
    }
}
