package frc.robot.commands;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InOrder;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import frc.robot.subsystems.OI;
import frc.robot.subsystems.ShooterHood;

import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;
import static org.mockito.Mockito.atLeastOnce;

@ExtendWith(MockitoExtension.class)
public class TestHoodTest
{
  @Mock
  private ShooterHood mockShooterHood;

  @Mock
  private OI mockOI;

  @Test
  public void testInitialize_setsLevelToZero()
  {
    TestHood cmd = new TestHood(mockShooterHood, mockOI);
    cmd.initialize();
    verify(mockShooterHood, times(0)).setPosition(anyDouble());
  }

  @Test
  public void testExecute_leftBumperEdgeTriggeredIncrementsLevel()
  {
    TestHood cmd = new TestHood(mockShooterHood, mockOI);
    cmd.initialize();

    when(mockOI.getOperatorLeftBumper()).thenReturn(true);
    when(mockOI.getOperatorRightBumper()).thenReturn(false);

    cmd.execute();
    double expectedPosition = ShooterHood.maxPositionRadians - 1 * 0.015625;
    verify(mockShooterHood).setPosition(expectedPosition);

    cmd.execute();
    verify(mockShooterHood, times(2)).setPosition(expectedPosition);
  }

  @Test
  public void testExecute_holdingLeftBumperDoesNotRetrigger()
  {
    TestHood cmd = new TestHood(mockShooterHood, mockOI);
    cmd.initialize();

    when(mockOI.getOperatorLeftBumper()).thenReturn(true);
    when(mockOI.getOperatorRightBumper()).thenReturn(false);

    cmd.execute();
    cmd.execute();
    cmd.execute();

    double expectedPosition = ShooterHood.maxPositionRadians - 1 * 0.015625;
    verify(mockShooterHood, times(3)).setPosition(expectedPosition);
  }

  @Test
  public void testExecute_rightBumperEdgeTriggeredDecrementsLevel()
  {
    TestHood cmd = new TestHood(mockShooterHood, mockOI);
    cmd.initialize();

    when(mockOI.getOperatorLeftBumper()).thenReturn(true);
    when(mockOI.getOperatorRightBumper()).thenReturn(false);
    cmd.execute();
    cmd.execute();

    when(mockOI.getOperatorLeftBumper()).thenReturn(false);
    when(mockOI.getOperatorRightBumper()).thenReturn(true);
    cmd.execute();

    double expectedPosition = ShooterHood.maxPositionRadians - 0 * 0.015625;
    verify(mockShooterHood).setPosition(expectedPosition);
  }

  @Test
  public void testExecute_levelClampedToMin()
  {
    TestHood cmd = new TestHood(mockShooterHood, mockOI);
    cmd.initialize();

    when(mockOI.getOperatorLeftBumper()).thenReturn(false);
    when(mockOI.getOperatorRightBumper()).thenReturn(true);

    cmd.execute();
    verify(mockShooterHood).setPosition(ShooterHood.maxPositionRadians);
  }

  @Test
  public void testExecute_levelClampedToMax()
  {
    TestHood cmd = new TestHood(mockShooterHood, mockOI);
    cmd.initialize();

    when(mockOI.getOperatorRightBumper()).thenReturn(false);

    for (int i = 0; i < 33; i++) {
      when(mockOI.getOperatorLeftBumper()).thenReturn(true);
      cmd.execute();
      when(mockOI.getOperatorLeftBumper()).thenReturn(false);
      cmd.execute();
    }

    double expectedPosition = ShooterHood.maxPositionRadians - 32 * 0.015625;
    verify(mockShooterHood, atLeastOnce()).setPosition(expectedPosition);
  }

  @Test
  public void testEnd_callsStop()
  {
    TestHood cmd = new TestHood(mockShooterHood, mockOI);

    InOrder inOrder = inOrder(mockShooterHood);
    cmd.end(false);
    inOrder.verify(mockShooterHood).stop();
  }

  @Test
  public void testIsFinished_alwaysFalse()
  {
    TestHood cmd = new TestHood(mockShooterHood, mockOI);
    assert !cmd.isFinished();
  }
}
