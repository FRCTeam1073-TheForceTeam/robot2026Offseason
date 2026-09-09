package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.inOrder;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.Test;
import org.mockito.InOrder;
import frc.robot.subsystems.ShooterHood;

public class ZeroHoodTest {
  private static final double LIMIT_NM = 2.2;

  @Test
  public void testIsFinished_belowThreshold() {
    ShooterHood mockHood = mock(ShooterHood.class);
    when(mockHood.getTorqueNm()).thenReturn(2.0);

    ZeroHood command = new ZeroHood(mockHood);

    assertFalse(command.isFinished());
  }

  @Test
  public void testIsFinished_aboveThreshold() {
    ShooterHood mockHood = mock(ShooterHood.class);
    when(mockHood.getTorqueNm()).thenReturn(3.0);

    ZeroHood command = new ZeroHood(mockHood);

    assertTrue(command.isFinished());
  }

  @Test
  public void testIsFinished_exactlyAtThreshold_returnsFalse() {
    ShooterHood mockHood = mock(ShooterHood.class);
    when(mockHood.getTorqueNm()).thenReturn(LIMIT_NM);

    ZeroHood command = new ZeroHood(mockHood);

    assertFalse(command.isFinished());
  }

  @Test
  public void testExecute_setsVelocity() {
    ShooterHood mockHood = mock(ShooterHood.class);
    when(mockHood.getTorqueNm()).thenReturn(0.0);

    ZeroHood command = new ZeroHood(mockHood);
    command.execute();

    verify(mockHood).setVelocity(0.2);
  }

  @Test
  public void testEnd_notInterrupted_callsZeroThenStop() {
    ShooterHood mockHood = mock(ShooterHood.class);

    ZeroHood command = new ZeroHood(mockHood);
    command.end(false);

    InOrder inOrder = inOrder(mockHood);
    inOrder.verify(mockHood).zero();
    inOrder.verify(mockHood).stop();
  }

  @Test
  public void testEnd_interrupted_callsZeroThenStop() {
    ShooterHood mockHood = mock(ShooterHood.class);

    ZeroHood command = new ZeroHood(mockHood);
    command.end(true);

    InOrder inOrder = inOrder(mockHood);
    inOrder.verify(mockHood).zero();
    inOrder.verify(mockHood).stop();
  }

  @Test
  public void testConstructor_addsRequirements() {
    ShooterHood mockHood = mock(ShooterHood.class);

    ZeroHood command = new ZeroHood(mockHood);

    assertTrue(command.getRequirements().contains(mockHood));
  }
}
