package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import java.util.Set;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests ZoneFinder's field geometry: which zones the localized pose falls in, with TRENCH and BUMP regions checked
 * against the blue-alliance layout. Poses are given in inches to match the zone definitions.
 *
 * periodic() must not throw when the driver station has not reported an alliance, including inside a trench.
 */
public class ZoneFinderTest {
    private FakeLocalizer localizer;
    private ZoneFinder zoneFinder;

    /** Localizer double whose pose is set directly by each test. */
    static class FakeLocalizer extends Localizer {
        Pose2d fakePose = new Pose2d();

        FakeLocalizer() {
            super(new LocalizerTest.MockDrivetrain(),
                new LocalizerTest.MockAprilTagFinder(new LocalizerTest.MockTurret()));
        }

        @Override
        public Pose2d getPose() {
            return fakePose;
        }
    }

    @BeforeEach
    public void setUp() {
        localizer = new FakeLocalizer();
        zoneFinder = new ZoneFinder(localizer);
        setAlliance(AllianceStationID.Blue1, Alliance.Blue);
    }

    @AfterEach
    public void tearDown() {
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
    }

    @Test
    public void testLowerBlueTrenchReportsTrenchAndBlueZone() {
        Set<String> zones = zonesAt(150, 25);

        assertTrue(zones.contains("TRENCH"), "Point in the lower trench should report TRENCH");
        assertTrue(zones.contains("BLUEZONE"), "Point in the lower blue trench should also report BLUEZONE");
    }

    @Test
    public void testUpperBlueTrenchReportsTrench() {
        assertTrue(zonesAt(150, 300).contains("TRENCH"),
            "Point in the upper trench should report TRENCH");
    }

    @Test
    public void testLowerRedTrenchReportsTrench() {
        assertTrue(zonesAt(450, 25).contains("TRENCH"),
            "Point in the lower red-side trench should report TRENCH");
    }

    @Test
    public void testUpperRedTrenchReportsTrench() {
        assertTrue(zonesAt(450, 300).contains("TRENCH"),
            "Point in the upper red-side trench should report TRENCH");
    }

    @Test
    public void testOpenFieldReportsNoTrench() {
        assertFalse(zonesAt(100, 150).contains("TRENCH"),
            "Point outside every trench rectangle should not report TRENCH");
    }

    @Test
    public void testPointJustInsideTrenchEdgeReportsTrench() {
        assertTrue(zonesAt(121.61 + 0.5, 25).contains("TRENCH"),
            "Point half an inch inside the trench's left edge should report TRENCH");
    }

    @Test
    public void testPointJustOutsideTrenchEdgeReportsNoTrench() {
        assertFalse(zonesAt(121.61 - 1.0, 25).contains("TRENCH"),
            "Point one inch outside the trench's left edge should not report TRENCH");
    }

    @Test
    public void testBumpIsNotTrench() {
        Set<String> zones = zonesAt(180, 100);

        assertTrue(zones.contains("BUMP"), "Point in the bump should report BUMP");
        assertFalse(zones.contains("TRENCH"), "Bump region should not report TRENCH");
    }

    @Test
    public void testHalfZonesSplitAtCenterline() {
        assertTrue(zonesAt(150, 100).contains("RIGHTHALF"), "Point below the centerline should be RIGHTHALF");
        assertTrue(zonesAt(150, 200).contains("LEFTHALF"), "Point above the centerline should be LEFTHALF");
    }

    @Test
    public void testPeriodicWithNoAllianceOffFieldDoesNotThrow() {
        clearAlliance();
        localizer.fakePose = inches(700, 400);

        assertDoesNotThrow(zoneFinder::periodic,
            "periodic() should not throw when no alliance is reported and the robot is in no zone");
        assertTrue(zoneFinder.getZones().isEmpty(), "Off-field point should report no zones");
    }

    @Test
    public void testPeriodicWithNoAllianceInsideTrenchDoesNotThrow() {
        // No alliance is present, so periodic() must not assume one when it swaps the left and right halves.
        clearAlliance();
        localizer.fakePose = inches(150, 25);

        assertDoesNotThrow(zoneFinder::periodic,
            "periodic() should not throw when no alliance is reported and the robot is inside a trench");
    }

    /** Sets the pose to the given inches, runs periodic() so the zones update, and returns the zones reported. */
    private Set<String> zonesAt(double xInches, double yInches) {
        localizer.fakePose = inches(xInches, yInches);
        zoneFinder.periodic();
        return zoneFinder.getZones();
    }

    private static Pose2d inches(double xInches, double yInches) {
        return new Pose2d(Units.inchesToMeters(xInches), Units.inchesToMeters(yInches), new Rotation2d());
    }

    private static void setAlliance(AllianceStationID station, Alliance expected) {
        DriverStationSim.setAllianceStationId(station);
        DriverStationSim.notifyNewData();
        assertEquals(expected, DriverStation.getAlliance().orElse(null),
            "DriverStationSim should make DriverStation report the alliance under test");
    }

    private static void clearAlliance() {
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
        assertTrue(DriverStation.getAlliance().isEmpty(),
            "DriverStationSim should make DriverStation report no alliance");
    }
}
