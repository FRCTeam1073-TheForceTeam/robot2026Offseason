package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.mockito.Mockito.mockStatic;

import java.util.Optional;
import java.util.Set;

import org.junit.jupiter.api.Test;
import org.mockito.MockedStatic;
import org.mockito.Mockito;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class TargetFinderTest {
    private static final double EPSILON = 1e-6;

    @Test
    public void testPass_redAllianceRightHalf() {
        try (MockedStatic<DriverStation> mockedDS = mockStatic(DriverStation.class)) {
            mockedDS.when(DriverStation::getAlliance).thenReturn(Optional.of(Alliance.Red));

            Localizer mockLocalizer = Mockito.mock(Localizer.class);
            ZoneFinder mockZoneFinder = Mockito.mock(ZoneFinder.class);
            TargetFinder targetFinder = new TargetFinder(mockLocalizer, mockZoneFinder);

            targetFinder.setZoneForTest(Set.of("RIGHTHALF"));
            targetFinder.setTurretPosForTest(new Pose2d());

            Pose2d result = targetFinder.pass();

            // Red alliance + RIGHTHALF should return REDPASS_L
            assertEquals(TargetFinder.REDPASS_L, result);
        }
    }

    @Test
    public void testPass_redAllianceLeftHalf() {
        try (MockedStatic<DriverStation> mockedDS = mockStatic(DriverStation.class)) {
            mockedDS.when(DriverStation::getAlliance).thenReturn(Optional.of(Alliance.Red));

            Localizer mockLocalizer = Mockito.mock(Localizer.class);
            ZoneFinder mockZoneFinder = Mockito.mock(ZoneFinder.class);
            TargetFinder targetFinder = new TargetFinder(mockLocalizer, mockZoneFinder);

            targetFinder.setZoneForTest(Set.of("LEFTHALF"));
            targetFinder.setTurretPosForTest(new Pose2d());

            Pose2d result = targetFinder.pass();

            // Red alliance + LEFTHALF should return REDPASS_R (swapped)
            assertEquals(TargetFinder.REDPASS_R, result);
        }
    }

    @Test
    public void testPass_blueAllianceRightHalf() {
        try (MockedStatic<DriverStation> mockedDS = mockStatic(DriverStation.class)) {
            mockedDS.when(DriverStation::getAlliance).thenReturn(Optional.of(Alliance.Blue));

            Localizer mockLocalizer = Mockito.mock(Localizer.class);
            ZoneFinder mockZoneFinder = Mockito.mock(ZoneFinder.class);
            TargetFinder targetFinder = new TargetFinder(mockLocalizer, mockZoneFinder);

            targetFinder.setZoneForTest(Set.of("RIGHTHALF"));
            targetFinder.setTurretPosForTest(new Pose2d());

            Pose2d result = targetFinder.pass();

            // Blue alliance + RIGHTHALF should return BLUEPASS_R (no swap)
            assertEquals(TargetFinder.BLUEPASS_R, result);
        }
    }

    @Test
    public void testPass_blueAllianceLeftHalf() {
        try (MockedStatic<DriverStation> mockedDS = mockStatic(DriverStation.class)) {
            mockedDS.when(DriverStation::getAlliance).thenReturn(Optional.of(Alliance.Blue));

            Localizer mockLocalizer = Mockito.mock(Localizer.class);
            ZoneFinder mockZoneFinder = Mockito.mock(ZoneFinder.class);
            TargetFinder targetFinder = new TargetFinder(mockLocalizer, mockZoneFinder);

            targetFinder.setZoneForTest(Set.of("LEFTHALF"));
            targetFinder.setTurretPosForTest(new Pose2d());

            Pose2d result = targetFinder.pass();

            // Blue alliance + LEFTHALF should return BLUEPASS_L
            assertEquals(TargetFinder.BLUEPASS_L, result);
        }
    }

    @Test
    public void testPass_allianceUnknown() {
        try (MockedStatic<DriverStation> mockedDS = mockStatic(DriverStation.class)) {
            mockedDS.when(DriverStation::getAlliance).thenReturn(Optional.empty());

            Localizer mockLocalizer = Mockito.mock(Localizer.class);
            ZoneFinder mockZoneFinder = Mockito.mock(ZoneFinder.class);
            TargetFinder targetFinder = new TargetFinder(mockLocalizer, mockZoneFinder);

            Pose2d turretPos = new Pose2d(1.0, 2.0, new Rotation2d());
            targetFinder.setTurretPosForTest(turretPos);

            Pose2d result = targetFinder.pass();

            // No alliance should default to FIELD_CENTER relative to turret
            Pose2d expected = TargetFinder.FIELD_CENTER.relativeTo(turretPos);
            assertEquals(expected.getX(), result.getX(), EPSILON);
            assertEquals(expected.getY(), result.getY(), EPSILON);
        }
    }

    @Test
    public void testPass_allianceKnownButZoneNeitherHalf() {
        try (MockedStatic<DriverStation> mockedDS = mockStatic(DriverStation.class)) {
            mockedDS.when(DriverStation::getAlliance).thenReturn(Optional.of(Alliance.Blue));

            Localizer mockLocalizer = Mockito.mock(Localizer.class);
            ZoneFinder mockZoneFinder = Mockito.mock(ZoneFinder.class);
            TargetFinder targetFinder = new TargetFinder(mockLocalizer, mockZoneFinder);

            targetFinder.setZoneForTest(Set.of("TRENCH"));
            targetFinder.setTurretPosForTest(new Pose2d());

            Pose2d result = targetFinder.pass();

            // Zone doesn't contain RIGHTHALF or LEFTHALF, so should default to FIELD_CENTER
            assertEquals(TargetFinder.FIELD_CENTER, result);
        }
    }

    @Test
    public void testGetHubPos_zeroVelocity() {
        Localizer mockLocalizer = Mockito.mock(Localizer.class);
        Mockito.when(mockLocalizer.getSpeeds()).thenReturn(new ChassisSpeeds(0, 0, 0));

        BallisticShot mockBallisticShot = Mockito.mock(BallisticShot.class);
        Mockito.when(mockBallisticShot.getShot()).thenReturn(new BallisticShot.Shot(8.0, 1.0, 1.0));

        ZoneFinder mockZoneFinder = Mockito.mock(ZoneFinder.class);
        TargetFinder targetFinder = new TargetFinder(mockLocalizer, mockZoneFinder);
        targetFinder.setBallisticShot(mockBallisticShot);

        targetFinder.setOurHubForTest(new Pose2d(1.0, 2.0, new Rotation2d()));
        targetFinder.setTurretPosForTest(new Pose2d(0.5, 1.5, new Rotation2d()));

        Pose2d result = targetFinder.getHubPos();

        // With zero velocity, should return hub relative to turret
        Pose2d expected = new Pose2d(1.0, 2.0, new Rotation2d()).relativeTo(new Pose2d(0.5, 1.5, new Rotation2d()));
        assertEquals(expected.getX(), result.getX(), EPSILON);
        assertEquals(expected.getY(), result.getY(), EPSILON);
    }

    @Test
    public void testGetTargetPos_ourZoneActive() {
        Localizer mockLocalizer = Mockito.mock(Localizer.class);
        Mockito.when(mockLocalizer.getSpeeds()).thenReturn(new ChassisSpeeds(0, 0, 0));

        BallisticShot mockBallisticShot = Mockito.mock(BallisticShot.class);
        Mockito.when(mockBallisticShot.getShot()).thenReturn(new BallisticShot.Shot(8.0, 1.0, 1.0));

        ZoneFinder mockZoneFinder = Mockito.mock(ZoneFinder.class);
        Mockito.when(mockZoneFinder.getZones()).thenReturn(Set.of("BLUEZONE", "RIGHTHALF"));

        TargetFinder targetFinder = new TargetFinder(mockLocalizer, mockZoneFinder);
        targetFinder.setBallisticShot(mockBallisticShot);

        targetFinder.setOurZoneForTest("BLUEZONE");
        targetFinder.setOurHubForTest(TargetFinder.BLUEHUB);
        targetFinder.setTurretPosForTest(new Pose2d());

        Pose2d result = targetFinder.getTargetPos();

        // Our zone is active, so should delegate to getHubPos
        assertFalse(targetFinder.isPassing());
    }
}
