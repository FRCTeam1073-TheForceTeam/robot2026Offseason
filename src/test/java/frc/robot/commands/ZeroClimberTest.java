package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.inOrder;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.Test;
import org.mockito.InOrder;
import frc.robot.subsystems.Climber;

public class ZeroClimberTest {
  private static final double LIMIT_N = 6.0;

  @Test
  public void testIsFinished_belowThreshold() {
    Climber mockClimber = mock(Climber.class);
    when(mockClimber.getForce()).thenReturn(5.0);

    ZeroClimber command = new ZeroClimber(mockClimber);

    assertFalse(command.isFinished());
  }

  @Test
  public void testIsFinished_aboveThreshold() {
    Climber mockClimber = mock(Climber.class);
    when(mockClimber.getForce()).thenReturn(7.0);

    ZeroClimber command = new ZeroClimber(mockClimber);

    assertTrue(command.isFinished());
  }

  @Test
  public void testIsFinished_exactlyAtThreshold_returnsFalse() {
    Climber mockClimber = mock(Climber.class);
    when(mockClimber.getForce()).thenReturn(LIMIT_N);

    ZeroClimber command = new ZeroClimber(mockClimber);

    assertFalse(command.isFinished());
  }

  @Test
  public void testIsFinished_negativeThreshold() {
    Climber mockClimber = mock(Climber.class);
    when(mockClimber.getForce()).thenReturn(-7.0);

    ZeroClimber command = new ZeroClimber(mockClimber);

    assertTrue(command.isFinished());
  }

  @Test
  public void testExecute_setsVelocity() {
    Climber mockClimber = mock(Climber.class);
    when(mockClimber.getForce()).thenReturn(0.0);

    ZeroClimber command = new ZeroClimber(mockClimber);
    command.execute();

    verify(mockClimber).setVelocity(-0.03);
  }

  @Test
  public void testEnd_notInterrupted_callsZeroThenStop() {
    Climber mockClimber = mock(Climber.class);

    ZeroClimber command = new ZeroClimber(mockClimber);
    command.end(false);

    InOrder inOrder = inOrder(mockClimber);
    inOrder.verify(mockClimber).zero();
    inOrder.verify(mockClimber).stop();
  }

  @Test
  public void testEnd_interrupted_callsZeroThenStop() {
    Climber mockClimber = mock(Climber.class);

    ZeroClimber command = new ZeroClimber(mockClimber);
    command.end(true);

    InOrder inOrder = inOrder(mockClimber);
    inOrder.verify(mockClimber).zero();
    inOrder.verify(mockClimber).stop();
  }

  @Test
  public void testConstructor_default_addsRequirements() {
    Climber mockClimber = mock(Climber.class);

    ZeroClimber command = new ZeroClimber(mockClimber);

    assertTrue(command.getRequirements().contains(mockClimber));
  }

  @Test
  public void testConstructor_unsafe_skipsRequirements() {
    Climber mockClimber = mock(Climber.class);

    ZeroClimber command = new ZeroClimber(mockClimber, true);

    assertTrue(command.getRequirements().isEmpty());
  }
}
