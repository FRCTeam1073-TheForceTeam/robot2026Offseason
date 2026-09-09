package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import frc.robot.subsystems.BallisticShot;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.TargetFinder;
import frc.robot.utilities.ShooterTable;

import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class FlywheelTeleopTest
{
  @Mock
  private Flywheel mockFlywheel;

  @Mock
  private OI mockOI;

  @Mock
  private TargetFinder mockTargetFinder;

  @Mock
  private ShooterTable mockShooterTable;

  @Mock
  private BallisticShot mockBallisticShot;

  @Test
  public void testExecute_noTriggerStops()
  {
    FlywheelTeleop cmd = new FlywheelTeleop(mockFlywheel, mockOI, mockTargetFinder, mockShooterTable, mockBallisticShot);

    when(mockOI.getOperatorLeftTrigger()).thenReturn(0.0);
    when(mockOI.getOperatorYButton()).thenReturn(false);
    when(mockOI.getOperatorXButton()).thenReturn(false);

    cmd.execute();
    verify(mockFlywheel).stop();
  }

  @Test
  public void testExecute_leftTriggerWithPassingModeLowRange()
  {
    FlywheelTeleop cmd = new FlywheelTeleop(mockFlywheel, mockOI, mockTargetFinder, mockShooterTable, mockBallisticShot);

    when(mockOI.getOperatorLeftTrigger()).thenReturn(0.5);
    when(mockTargetFinder.isPassing()).thenReturn(true);
    when(mockTargetFinder.getRangeToTargetMeters()).thenReturn(Units.inchesToMeters(250));

    cmd.execute();
    verify(mockFlywheel).setVelocity(9.0);
  }

  @Test
  public void testExecute_leftTriggerWithPassingModeHighRange()
  {
    FlywheelTeleop cmd = new FlywheelTeleop(mockFlywheel, mockOI, mockTargetFinder, mockShooterTable, mockBallisticShot);

    when(mockOI.getOperatorLeftTrigger()).thenReturn(0.5);
    when(mockTargetFinder.isPassing()).thenReturn(true);
    when(mockTargetFinder.getRangeToTargetMeters()).thenReturn(Units.inchesToMeters(300));

    cmd.execute();
    verify(mockFlywheel).setVelocity(15.0);
  }

  @Test
  public void testExecute_leftTriggerWithBallisticShotMode()
  {
    FlywheelTeleop cmd = new FlywheelTeleop(mockFlywheel, mockOI, mockTargetFinder, mockShooterTable, mockBallisticShot);

    when(mockOI.getOperatorLeftTrigger()).thenReturn(0.5);
    when(mockTargetFinder.isPassing()).thenReturn(false);
    when(mockOI.ballisticShotMode()).thenReturn(true);
    when(mockBallisticShot.getShot()).thenReturn(new BallisticShot.Shot(12.5, 0.5, 2.0));

    cmd.execute();
    verify(mockFlywheel).setVelocity(12.5);
  }

  @Test
  public void testExecute_leftTriggerWithTableMode()
  {
    FlywheelTeleop cmd = new FlywheelTeleop(mockFlywheel, mockOI, mockTargetFinder, mockShooterTable, mockBallisticShot);

    when(mockOI.getOperatorLeftTrigger()).thenReturn(0.5);
    when(mockTargetFinder.isPassing()).thenReturn(false);
    when(mockOI.ballisticShotMode()).thenReturn(false);
    when(mockTargetFinder.getRangeToTargetMeters()).thenReturn(2.0);
    when(mockShooterTable.getFlywheelVelocity(2.0)).thenReturn(11.0);

    cmd.execute();
    verify(mockFlywheel).setVelocity(11.0);
  }

  @Test
  public void testExecute_yButtonCornerShot()
  {
    FlywheelTeleop cmd = new FlywheelTeleop(mockFlywheel, mockOI, mockTargetFinder, mockShooterTable, mockBallisticShot);

    when(mockOI.getOperatorLeftTrigger()).thenReturn(0.0);
    when(mockOI.getOperatorYButton()).thenReturn(true);
    when(mockOI.getOperatorXButton()).thenReturn(false);

    cmd.execute();
    verify(mockFlywheel).setVelocity(9.8);
  }

  @Test
  public void testExecute_xButtonTowerShot()
  {
    FlywheelTeleop cmd = new FlywheelTeleop(mockFlywheel, mockOI, mockTargetFinder, mockShooterTable, mockBallisticShot);

    when(mockOI.getOperatorLeftTrigger()).thenReturn(0.0);
    when(mockOI.getOperatorYButton()).thenReturn(false);
    when(mockOI.getOperatorXButton()).thenReturn(true);

    cmd.execute();
    verify(mockFlywheel).setVelocity(9.2);
  }

  @Test
  public void testEnd_callsStop()
  {
    FlywheelTeleop cmd = new FlywheelTeleop(mockFlywheel, mockOI, mockTargetFinder, mockShooterTable, mockBallisticShot);
    cmd.end(false);
    verify(mockFlywheel).stop();
  }

  @Test
  public void testIsFinished_alwaysFalse()
  {
    FlywheelTeleop cmd = new FlywheelTeleop(mockFlywheel, mockOI, mockTargetFinder, mockShooterTable, mockBallisticShot);
    assert !cmd.isFinished();
  }
}
