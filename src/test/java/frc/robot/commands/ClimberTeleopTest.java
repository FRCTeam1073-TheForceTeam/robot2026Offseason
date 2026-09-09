package frc.robot.commands;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.ZoneFinder;

import java.util.HashSet;
import java.util.Set;

import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class ClimberTeleopTest
{
  @Mock
  private Climber mockClimber;

  @Mock
  private OI mockOI;

  @Mock
  private ZoneFinder mockZoneFinder;

  @Test
  public void testInitialize_storesCurrentPosition()
  {
    when(mockClimber.getClimberPosition()).thenReturn(0.03);

    ClimberTeleop cmd = new ClimberTeleop(mockClimber, mockOI, mockZoneFinder);
    cmd.initialize();

    Set<String> noZone = new HashSet<>();
    when(mockZoneFinder.getZones()).thenReturn(noZone);
    when(mockOI.getDriverMenuButton()).thenReturn(false);
    when(mockOI.getDriverViewButton()).thenReturn(false);

    cmd.execute();
    verify(mockClimber).setPosition(0.03);
  }

  @Test
  public void testExecute_menuButtonIncreasesPosition()
  {
    when(mockClimber.getClimberPosition()).thenReturn(0.03);

    ClimberTeleop cmd = new ClimberTeleop(mockClimber, mockOI, mockZoneFinder);
    cmd.initialize();

    Set<String> noZone = new HashSet<>();
    when(mockZoneFinder.getZones()).thenReturn(noZone);
    when(mockOI.getDriverMenuButton()).thenReturn(true);
    when(mockOI.getDriverViewButton()).thenReturn(false);
    when(mockOI.getOperatorViewButton()).thenReturn(false);
    when(mockOI.getOperatorMenuButton()).thenReturn(false);

    cmd.execute();
    verify(mockClimber).setPosition(0.03 + 0.1);
  }

  @Test
  public void testExecute_viewButtonDecreasesPosition()
  {
    when(mockClimber.getClimberPosition()).thenReturn(0.04);

    ClimberTeleop cmd = new ClimberTeleop(mockClimber, mockOI, mockZoneFinder);
    cmd.initialize();

    Set<String> noZone = new HashSet<>();
    when(mockZoneFinder.getZones()).thenReturn(noZone);
    when(mockOI.getDriverMenuButton()).thenReturn(false);
    when(mockOI.getDriverViewButton()).thenReturn(true);
    when(mockOI.getOperatorViewButton()).thenReturn(false);
    when(mockOI.getOperatorMenuButton()).thenReturn(false);

    cmd.execute();
    verify(mockClimber).setPosition(0.04 - 0.1);
  }

  @Test
  public void testExecute_operatorViewButtonSetsMaxPosition()
  {
    when(mockClimber.getClimberPosition()).thenReturn(0.03);

    ClimberTeleop cmd = new ClimberTeleop(mockClimber, mockOI, mockZoneFinder);
    cmd.initialize();

    Set<String> noZone = new HashSet<>();
    when(mockZoneFinder.getZones()).thenReturn(noZone);
    when(mockOI.getDriverMenuButton()).thenReturn(false);
    when(mockOI.getDriverViewButton()).thenReturn(false);
    when(mockOI.getOperatorViewButton()).thenReturn(true);
    when(mockOI.getOperatorMenuButton()).thenReturn(false);

    cmd.execute();
    verify(mockClimber).setPosition(0.0582);
  }

  @Test
  public void testExecute_operatorMenuButtonSetsMinPosition()
  {
    when(mockClimber.getClimberPosition()).thenReturn(0.05);

    ClimberTeleop cmd = new ClimberTeleop(mockClimber, mockOI, mockZoneFinder);
    cmd.initialize();

    Set<String> noZone = new HashSet<>();
    when(mockZoneFinder.getZones()).thenReturn(noZone);
    when(mockOI.getDriverMenuButton()).thenReturn(false);
    when(mockOI.getDriverViewButton()).thenReturn(false);
    when(mockOI.getOperatorViewButton()).thenReturn(false);
    when(mockOI.getOperatorMenuButton()).thenReturn(true);

    cmd.execute();
    verify(mockClimber).setPosition(0.0);
  }

  @Test
  public void testExecute_trenchZoneOverridesButtons()
  {
    when(mockClimber.getClimberPosition()).thenReturn(0.05);

    ClimberTeleop cmd = new ClimberTeleop(mockClimber, mockOI, mockZoneFinder);
    cmd.initialize();

    Set<String> trenchZone = new HashSet<>();
    trenchZone.add("TRENCH");
    when(mockZoneFinder.getZones()).thenReturn(trenchZone);
    when(mockOI.getDriverMenuButton()).thenReturn(true);
    when(mockOI.getDriverViewButton()).thenReturn(false);
    when(mockOI.getOperatorViewButton()).thenReturn(false);
    when(mockOI.getOperatorMenuButton()).thenReturn(false);

    cmd.execute();
    verify(mockClimber).setPosition(0.0);
  }

  @Test
  public void testEnd_callsStop()
  {
    ClimberTeleop cmd = new ClimberTeleop(mockClimber, mockOI, mockZoneFinder);
    cmd.end(false);
    verify(mockClimber).stop();
  }

  @Test
  public void testIsFinished_alwaysFalse()
  {
    ClimberTeleop cmd = new ClimberTeleop(mockClimber, mockOI, mockZoneFinder);
    assert !cmd.isFinished();
  }
}
