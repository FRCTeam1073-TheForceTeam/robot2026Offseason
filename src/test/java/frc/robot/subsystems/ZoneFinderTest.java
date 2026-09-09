package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertFalse;

import java.util.Set;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeEach;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.mockito.Mockito;

public class ZoneFinderTest {
  private Localizer mockLocalizer;

  @BeforeEach
  public void setup() {
    mockLocalizer = Mockito.mock(Localizer.class);
  }

  private ZoneFinder createZoneFinder() {
    return new ZoneFinder(mockLocalizer);
  }

  private Translation2d inMeters(double xInches, double yInches) {
    return new Translation2d(Units.inchesToMeters(xInches), Units.inchesToMeters(yInches));
  }

  @Test
  public void testGetZones_pointInBluezone() {
    ZoneFinder zf = createZoneFinder();
    zf.setCurrentTranslationForTest(inMeters(50, 50));

    Set<String> zones = zf.getZones();

    assertTrue(zones.contains("BLUEZONE"));
  }

  @Test
  public void testGetZones_pointInNeutralZone() {
    ZoneFinder zf = createZoneFinder();
    zf.setCurrentTranslationForTest(inMeters(250, 150));

    Set<String> zones = zf.getZones();

    assertTrue(zones.contains("NEUTRALZONE"));
  }

  @Test
  public void testGetZones_pointInRedZone() {
    ZoneFinder zf = createZoneFinder();
    zf.setCurrentTranslationForTest(inMeters(550, 150));

    Set<String> zones = zf.getZones();

    assertTrue(zones.contains("REDZONE"));
  }

  @Test
  public void testGetZones_pointInRightHalf() {
    ZoneFinder zf = createZoneFinder();
    zf.setCurrentTranslationForTest(inMeters(100, 50));

    Set<String> zones = zf.getZones();

    assertTrue(zones.contains("RIGHTHALF"));
    assertFalse(zones.contains("LEFTHALF"));
  }

  @Test
  public void testGetZones_pointInLeftHalf() {
    ZoneFinder zf = createZoneFinder();
    zf.setCurrentTranslationForTest(inMeters(100, 200));

    Set<String> zones = zf.getZones();

    assertTrue(zones.contains("LEFTHALF"));
    assertFalse(zones.contains("RIGHTHALF"));
  }

  @Test
  public void testGetZones_pointOutsideAllZones() {
    ZoneFinder zf = createZoneFinder();
    zf.setCurrentTranslationForTest(new Translation2d(-10.0, -10.0));

    Set<String> zones = zf.getZones();

    assertTrue(zones.isEmpty());
  }

  @Test
  public void testGetZones_resultIsSorted() {
    ZoneFinder zf = createZoneFinder();
    zf.setCurrentTranslationForTest(inMeters(300, 100));

    Set<String> zones = zf.getZones();

    // Should be a TreeSet, so zones are naturally ordered
    assertFalse(zones.isEmpty());
  }

  @Test
  public void testGetZones_multipleMembership() {
    ZoneFinder zf = createZoneFinder();
    // Point that's in both BLUEZONE and RIGHTHALF
    zf.setCurrentTranslationForTest(inMeters(100, 50));

    Set<String> zones = zf.getZones();

    assertTrue(zones.contains("BLUEZONE"));
    assertTrue(zones.contains("RIGHTHALF"));
  }
}
