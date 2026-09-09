package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.inOrder;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.Test;
import org.mockito.InOrder;
import frc.robot.subsystems.Turret;

public class ZeroTurretTest {
  private static final double LIMIT_NM = 3.5;

  @Test
  public void testIsFinished_belowThreshold() {
    Turret mockTurret = mock(Turret.class);
    when(mockTurret.getTorqueNm()).thenReturn(3.0);

    ZeroTurret command = new ZeroTurret(mockTurret);

    assertFalse(command.isFinished());
  }

  @Test
  public void testIsFinished_aboveThreshold() {
    Turret mockTurret = mock(Turret.class);
    when(mockTurret.getTorqueNm()).thenReturn(4.0);

    ZeroTurret command = new ZeroTurret(mockTurret);

    assertTrue(command.isFinished());
  }

  @Test
  public void testIsFinished_exactlyAtThreshold_returnsFalse() {
    Turret mockTurret = mock(Turret.class);
    when(mockTurret.getTorqueNm()).thenReturn(LIMIT_NM);

    ZeroTurret command = new ZeroTurret(mockTurret);

    // Exactly at threshold uses strict >, so returns false
    assertFalse(command.isFinished());
  }

  @Test
  public void testIsFinished_negativeThreshold() {
    Turret mockTurret = mock(Turret.class);
    when(mockTurret.getTorqueNm()).thenReturn(-4.0);

    ZeroTurret command = new ZeroTurret(mockTurret);

    // Negative torque below -3.5 is still outside the range, so abs(-4.0) > 3.5
    assertTrue(command.isFinished());
  }

  @Test
  public void testExecute_setsVelocity() {
    Turret mockTurret = mock(Turret.class);
    when(mockTurret.getTorqueNm()).thenReturn(0.0);

    ZeroTurret command = new ZeroTurret(mockTurret);
    command.execute();

    verify(mockTurret).setVelocity(2.0);
  }

  @Test
  public void testEnd_notInterrupted_callsZeroThenStop() {
    Turret mockTurret = mock(Turret.class);

    ZeroTurret command = new ZeroTurret(mockTurret);
    command.end(false);

    InOrder inOrder = inOrder(mockTurret);
    inOrder.verify(mockTurret).zero();
    inOrder.verify(mockTurret).stop();
  }

  @Test
  public void testEnd_interrupted_callsZeroThenStop() {
    Turret mockTurret = mock(Turret.class);

    ZeroTurret command = new ZeroTurret(mockTurret);
    command.end(true);

    InOrder inOrder = inOrder(mockTurret);
    inOrder.verify(mockTurret).zero();
    inOrder.verify(mockTurret).stop();
  }

  @Test
  public void testConstructor_default_addsRequirements() {
    Turret mockTurret = mock(Turret.class);

    ZeroTurret command = new ZeroTurret(mockTurret);

    // getRequirements() should contain the turret
    assertTrue(command.getRequirements().contains(mockTurret));
  }

  @Test
  public void testConstructor_unsafe_skipsRequirements() {
    Turret mockTurret = mock(Turret.class);

    ZeroTurret command = new ZeroTurret(mockTurret, true);

    // getRequirements() should be empty when unsafe=true
    assertTrue(command.getRequirements().isEmpty());
  }
}
