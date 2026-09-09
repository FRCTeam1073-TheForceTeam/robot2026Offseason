package frc.robot.commands;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.ZoneFinder;

import java.util.HashSet;
import java.util.Set;

import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class IntakeTeleopTest
{
  @Mock
  private Intake mockIntake;

  @Mock
  private OI mockOI;

  @Mock
  private ZoneFinder mockZoneFinder;

  @Test
  public void testExecute_initialStatePositionIn()
  {
    when(mockIntake.getPositionRadians()).thenReturn(0.0);

    IntakeTeleop cmd = new IntakeTeleop(mockIntake, mockOI, mockZoneFinder);

    Set<String> noZone = new HashSet<>();
    when(mockZoneFinder.getZones()).thenReturn(noZone);
    when(mockOI.getDriverRightBumper()).thenReturn(false);

    cmd.execute();
    verify(mockIntake).setPosition(Math.toRadians(-122.0));
  }

  @Test
  public void testExecute_rightBumperEdgeTriggeredToggles()
  {
    when(mockIntake.getPositionRadians()).thenReturn(0.0);

    IntakeTeleop cmd = new IntakeTeleop(mockIntake, mockOI, mockZoneFinder);

    Set<String> noZone = new HashSet<>();
    when(mockZoneFinder.getZones()).thenReturn(noZone);

    when(mockOI.getDriverRightBumper()).thenReturn(false);
    cmd.execute();
    verify(mockIntake).setPosition(Math.toRadians(-122.0));

    when(mockOI.getDriverRightBumper()).thenReturn(true);
    when(mockIntake.getPositionRadians()).thenReturn(Math.toRadians(-122.0));
    cmd.execute();
    verify(mockIntake).setPosition(Math.toRadians(-0.1));
  }

  @Test
  public void testExecute_holdingBumperDoesNotRetoggle()
  {
    when(mockIntake.getPositionRadians()).thenReturn(0.0);

    IntakeTeleop cmd = new IntakeTeleop(mockIntake, mockOI, mockZoneFinder);

    Set<String> noZone = new HashSet<>();
    when(mockZoneFinder.getZones()).thenReturn(noZone);

    when(mockOI.getDriverRightBumper()).thenReturn(true);
    cmd.execute();

    when(mockIntake.getPositionRadians()).thenReturn(Math.toRadians(-122.0));
    cmd.execute();

    when(mockIntake.getPositionRadians()).thenReturn(Math.toRadians(-0.1));
    cmd.execute();

    verify(mockIntake).setPosition(Math.toRadians(-0.1));
  }

  @Test
  public void testExecute_trenchZoneForcesModeOut()
  {
    when(mockIntake.getPositionRadians()).thenReturn(1.0);

    IntakeTeleop cmd = new IntakeTeleop(mockIntake, mockOI, mockZoneFinder);

    Set<String> trenchZone = new HashSet<>();
    trenchZone.add("TRENCH");
    when(mockZoneFinder.getZones()).thenReturn(trenchZone);
    when(mockOI.getDriverRightBumper()).thenReturn(false);

    cmd.execute();
    verify(mockIntake).setPosition(Math.toRadians(-0.1));
  }

  @Test
  public void testExecute_trenchZoneAutoStopsWhenNearZero()
  {
    when(mockIntake.getPositionRadians()).thenReturn(0.15);

    IntakeTeleop cmd = new IntakeTeleop(mockIntake, mockOI, mockZoneFinder);

    Set<String> trenchZone = new HashSet<>();
    trenchZone.add("TRENCH");
    when(mockZoneFinder.getZones()).thenReturn(trenchZone);
    when(mockOI.getDriverRightBumper()).thenReturn(false);

    cmd.execute();
    verify(mockIntake).stop();
  }

  @Test
  public void testExecute_autoStopsWhenNearZero()
  {
    IntakeTeleop cmd = new IntakeTeleop(mockIntake, mockOI, mockZoneFinder);

    Set<String> noZone = new HashSet<>();
    when(mockZoneFinder.getZones()).thenReturn(noZone);

    when(mockOI.getDriverRightBumper()).thenReturn(false);
    when(mockIntake.getPositionRadians()).thenReturn(0.0);
    cmd.execute();
    verify(mockIntake).setPosition(Math.toRadians(-122.0));

    when(mockOI.getDriverRightBumper()).thenReturn(true);
    when(mockIntake.getPositionRadians()).thenReturn(0.3);
    cmd.execute();
    verify(mockIntake).setPosition(Math.toRadians(-0.1));

    when(mockOI.getDriverRightBumper()).thenReturn(false);
    when(mockIntake.getPositionRadians()).thenReturn(0.15);
    cmd.execute();
    verify(mockIntake).stop();
  }

  @Test
  public void testExecute_autoStopsWhenNegativeNearZero()
  {
    IntakeTeleop cmd = new IntakeTeleop(mockIntake, mockOI, mockZoneFinder);

    Set<String> noZone = new HashSet<>();
    when(mockZoneFinder.getZones()).thenReturn(noZone);

    when(mockOI.getDriverRightBumper()).thenReturn(false);
    when(mockIntake.getPositionRadians()).thenReturn(0.0);
    cmd.execute();
    verify(mockIntake).setPosition(Math.toRadians(-122.0));

    when(mockOI.getDriverRightBumper()).thenReturn(true);
    when(mockIntake.getPositionRadians()).thenReturn(0.3);
    cmd.execute();
    verify(mockIntake).setPosition(Math.toRadians(-0.1));

    when(mockOI.getDriverRightBumper()).thenReturn(false);
    when(mockIntake.getPositionRadians()).thenReturn(-0.05);
    cmd.execute();
    verify(mockIntake).stop();
  }

  @Test
  public void testEnd_callsStop()
  {
    IntakeTeleop cmd = new IntakeTeleop(mockIntake, mockOI, mockZoneFinder);
    cmd.end(false);
    verify(mockIntake).stop();
  }

  @Test
  public void testIsFinished_alwaysFalse()
  {
    IntakeTeleop cmd = new IntakeTeleop(mockIntake, mockOI, mockZoneFinder);
    assert !cmd.isFinished();
  }
}
