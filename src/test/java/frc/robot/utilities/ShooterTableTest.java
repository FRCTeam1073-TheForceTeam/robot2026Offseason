package frc.robot.utilities;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class ShooterTableTest {
  private static final double EPSILON = 1e-6;

  private ShooterTable table = new ShooterTable();

  @Test
  public void testGetHoodAngle_atFirstBreakpoint() {
    double result = table.getHoodAngle(1.0);
    double expected = Math.toRadians(69.2);
    assertEquals(expected, result, EPSILON);
  }

  @Test
  public void testGetHoodAngle_atSecondBreakpoint() {
    double result = table.getHoodAngle(3.0);
    double expected = Math.toRadians(61.17);
    assertEquals(expected, result, EPSILON);
  }

  @Test
  public void testGetHoodAngle_atLastBreakpoint() {
    double result = table.getHoodAngle(6.0);
    double expected = Math.toRadians(41.7);
    assertEquals(expected, result, EPSILON);
  }

  @Test
  public void testGetHoodAngle_interpolatesBetweenBreakpoints() {
    // Midpoint between 1.5 and 2.0
    double result = table.getHoodAngle(1.75);
    // Interpolating between ~65.46° and ~63.79° → ~64.625°
    double expected = Math.toRadians(64.625);
    assertEquals(expected, result, 0.01);
  }

  @Test
  public void testGetHoodAngle_clampsBeforeMin() {
    double result = table.getHoodAngle(0.5);
    double expected = Math.toRadians(69.2);
    assertEquals(expected, result, EPSILON);
  }

  @Test
  public void testGetHoodAngle_clampsAfterMax() {
    double result = table.getHoodAngle(10.0);
    double expected = Math.toRadians(41.7);
    assertEquals(expected, result, EPSILON);
  }

  @Test
  public void testGetFlywheelVelocity_atFirstBreakpoint() {
    double result = table.getFlywheelVelocity(1.0);
    double expected = 8.0 * 0.975;
    assertEquals(expected, result, EPSILON);
  }

  @Test
  public void testGetFlywheelVelocity_atUnscaledBreakpoint() {
    // 6.0 and 7.0 are NOT scaled by SPEED_SCALE
    double result = table.getFlywheelVelocity(6.0);
    assertEquals(9.0, result, EPSILON);
  }

  @Test
  public void testGetFlywheelVelocity_atAnotherUnscaledBreakpoint() {
    double result = table.getFlywheelVelocity(7.0);
    assertEquals(14.0, result, EPSILON);
  }

  @Test
  public void testGetFlywheelVelocity_interpolatesBetweenBreakpoints() {
    double result = table.getFlywheelVelocity(1.75);
    // Just verify it's between the two endpoints
    assertTrue(result > 8.2 && result < 8.7);
  }

  @Test
  public void testGetFlywheelVelocity_clampsBeforeMin() {
    double result = table.getFlywheelVelocity(0.5);
    double expected = 8.0 * 0.975;
    assertEquals(expected, result, EPSILON);
  }

  @Test
  public void testGetFlywheelVelocity_clampsAfterMax() {
    double result = table.getFlywheelVelocity(10.0);
    assertEquals(14.0, result, EPSILON);
  }
}
