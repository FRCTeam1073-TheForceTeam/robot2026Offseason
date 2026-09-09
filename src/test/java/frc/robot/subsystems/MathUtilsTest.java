package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import edu.wpi.first.math.filter.SlewRateLimiter;
import org.mockito.Mockito;

public class MathUtilsTest {
  private static final double EPSILON = 1e-9;

  // wrapAngleRadians tests
  @Test
  public void testWrapAngleRadians_identity() {
    assertEquals(0.0, MathUtils.wrapAngleRadians(0.0), EPSILON);
  }

  @Test
  public void testWrapAngleRadians_withinRange() {
    assertEquals(Math.PI / 4, MathUtils.wrapAngleRadians(Math.PI / 4), EPSILON);
  }

  @Test
  public void testWrapAngleRadians_positiveBoundary() {
    assertEquals(-Math.PI, MathUtils.wrapAngleRadians(Math.PI), EPSILON);
  }

  @Test
  public void testWrapAngleRadians_negativeBoundary() {
    assertEquals(-Math.PI, MathUtils.wrapAngleRadians(-Math.PI), EPSILON);
  }

  @Test
  public void testWrapAngleRadians_largePositiveWrap() {
    assertEquals(-Math.PI, MathUtils.wrapAngleRadians(3 * Math.PI), EPSILON);
  }

  @Test
  public void testWrapAngleRadians_270degrees() {
    assertEquals(-Math.PI / 2, MathUtils.wrapAngleRadians(3 * Math.PI / 2), EPSILON);
  }

  @Test
  public void testWrapAngleRadians_negativeWrap() {
    assertEquals(Math.PI / 2, MathUtils.wrapAngleRadians(-3 * Math.PI / 2), EPSILON);
  }

  @Test
  public void testWrapAngleRadians_largeNegativeWrap() {
    assertEquals(Math.PI / 4, MathUtils.wrapAngleRadians(-7 * Math.PI / 4), EPSILON);
  }

  @Test
  public void testWrapAngleRadians_arbitraryValue() {
    assertEquals(-0.7831853071795862, MathUtils.wrapAngleRadians(5.5), EPSILON);
  }

  // wrapAngleDegrees tests
  @Test
  public void testWrapAngleDegrees_identity() {
    assertEquals(0.0, MathUtils.wrapAngleDegrees(0.0), EPSILON);
  }

  @Test
  public void testWrapAngleDegrees_boundary() {
    assertEquals(-180.0, MathUtils.wrapAngleDegrees(180.0), EPSILON);
  }

  @Test
  public void testWrapAngleDegrees_negativeBoundary() {
    assertEquals(-180.0, MathUtils.wrapAngleDegrees(-180.0), EPSILON);
  }

  @Test
  public void testWrapAngleDegrees_multiWrap() {
    assertEquals(-180.0, MathUtils.wrapAngleDegrees(540.0), EPSILON);
  }

  @Test
  public void testWrapAngleDegrees_270() {
    assertEquals(-90.0, MathUtils.wrapAngleDegrees(270.0), EPSILON);
  }

  @Test
  public void testWrapAngleDegrees_negative270() {
    assertEquals(90.0, MathUtils.wrapAngleDegrees(-270.0), EPSILON);
  }

  @Test
  public void testWrapAngleDegrees_noWrap() {
    assertEquals(45.0, MathUtils.wrapAngleDegrees(45.0), EPSILON);
  }

  @Test
  public void testWrapAngleDegrees_negative315() {
    assertEquals(45.0, MathUtils.wrapAngleDegrees(-315.0), EPSILON);
  }

  @Test
  public void testWrapAngleDegrees_400() {
    assertEquals(40.0, MathUtils.wrapAngleDegrees(400.0), EPSILON);
  }

  // motorTurnsToRadians tests
  @Test
  public void testMotorTurnsToRadians_basicConversion() {
    double motorTurns = 1.0;
    double gearRatio = 50.0;
    double result = MathUtils.motorTurnsToRadians(motorTurns, gearRatio);
    double expected = 1.0 * 2.0 * Math.PI / 50.0;
    assertEquals(expected, result, EPSILON);
  }

  @Test
  public void testMotorTurnsToRadians_roundTrip() {
    double radians = Math.PI / 4;
    double gearRatio = 32.0;
    double turns = MathUtils.radiansToMotorTurns(radians, gearRatio);
    double backToRadians = MathUtils.motorTurnsToRadians(turns, gearRatio);
    assertEquals(radians, backToRadians, EPSILON);
  }

  // radiansToMotorTurns tests
  @Test
  public void testRadiansToMotorTurns_basicConversion() {
    double radians = Math.PI;
    double gearRatio = 40.0;
    double result = MathUtils.radiansToMotorTurns(radians, gearRatio);
    double expected = Math.PI * 40.0 / (2.0 * Math.PI);
    assertEquals(expected, result, EPSILON);
  }

  // motorTurnsToMeters tests
  @Test
  public void testMotorTurnsToMeters_basicConversion() {
    double motorTurns = 10.0;
    double turnsPerMeter = 100.0;
    double gearRatio = 2.0;
    double result = MathUtils.motorTurnsToMeters(motorTurns, turnsPerMeter, gearRatio);
    double expected = 10.0 / (100.0 * 2.0);
    assertEquals(expected, result, EPSILON);
  }

  @Test
  public void testMotorTurnsToMeters_roundTrip() {
    double meters = 0.5;
    double turnsPerMeter = 50.0;
    double gearRatio = 3.0;
    double turns = MathUtils.metersToMotorTurns(meters, turnsPerMeter, gearRatio);
    double backToMeters = MathUtils.motorTurnsToMeters(turns, turnsPerMeter, gearRatio);
    assertEquals(meters, backToMeters, EPSILON);
  }

  // metersToMotorTurns tests
  @Test
  public void testMetersToMotorTurns_basicConversion() {
    double meters = 1.0;
    double turnsPerMeter = 80.0;
    double gearRatio = 1.5;
    double result = MathUtils.metersToMotorTurns(meters, turnsPerMeter, gearRatio);
    double expected = 1.0 * 80.0 * 1.5;
    assertEquals(expected, result, EPSILON);
  }

  // currentToForce tests
  @Test
  public void testCurrentToForce_positive() {
    double amps = 5.0;
    double ampsPerUnit = 10.0;
    double result = MathUtils.currentToForce(amps, ampsPerUnit);
    assertEquals(0.5, result, EPSILON);
  }

  @Test
  public void testCurrentToForce_negative() {
    double amps = -5.0;
    double ampsPerUnit = 10.0;
    double result = MathUtils.currentToForce(amps, ampsPerUnit);
    assertEquals(-0.5, result, EPSILON);
  }

  @Test
  public void testCurrentToForce_zero() {
    assertEquals(0.0, MathUtils.currentToForce(0.0, 10.0), EPSILON);
  }

  // withinTolerance tests
  @Test
  public void testWithinTolerance_inside() {
    assertTrue(MathUtils.withinTolerance(10.0, 10.5, 1.0));
  }

  @Test
  public void testWithinTolerance_exactlyAtBoundary() {
    assertFalse(MathUtils.withinTolerance(10.0, 11.0, 1.0));
  }

  @Test
  public void testWithinTolerance_outside() {
    assertFalse(MathUtils.withinTolerance(10.0, 12.0, 1.0));
  }

  @Test
  public void testWithinTolerance_negativeDirection() {
    assertTrue(MathUtils.withinTolerance(10.0, 9.5, 1.0));
  }

  @Test
  public void testWithinTolerance_negativeDirectionAtBoundary() {
    assertFalse(MathUtils.withinTolerance(10.0, 9.0, 1.0));
  }

  // clampThenLimit tests
  @Test
  public void testClampThenLimit_targetWithinBounds() {
    SlewRateLimiter mockLimiter = Mockito.mock(SlewRateLimiter.class);
    Mockito.when(mockLimiter.calculate(5.0)).thenReturn(4.5);

    double result = MathUtils.clampThenLimit(mockLimiter, 5.0, 0.0, 10.0);

    assertEquals(4.5, result, EPSILON);
    Mockito.verify(mockLimiter).calculate(5.0);
  }

  @Test
  public void testClampThenLimit_targetAboveMax() {
    SlewRateLimiter mockLimiter = Mockito.mock(SlewRateLimiter.class);
    Mockito.when(mockLimiter.calculate(10.0)).thenReturn(9.5);

    double result = MathUtils.clampThenLimit(mockLimiter, 15.0, 0.0, 10.0);

    assertEquals(9.5, result, EPSILON);
    Mockito.verify(mockLimiter).calculate(10.0);
  }

  @Test
  public void testClampThenLimit_targetBelowMin() {
    SlewRateLimiter mockLimiter = Mockito.mock(SlewRateLimiter.class);
    Mockito.when(mockLimiter.calculate(0.0)).thenReturn(0.1);

    double result = MathUtils.clampThenLimit(mockLimiter, -5.0, 0.0, 10.0);

    assertEquals(0.1, result, EPSILON);
    Mockito.verify(mockLimiter).calculate(0.0);
  }

  // limitThenClamp tests
  @Test
  public void testLimitThenClamp_targetWithinBounds() {
    SlewRateLimiter mockLimiter = Mockito.mock(SlewRateLimiter.class);
    Mockito.when(mockLimiter.calculate(5.0)).thenReturn(5.0);

    double result = MathUtils.limitThenClamp(mockLimiter, 5.0, 0.0, 10.0);

    assertEquals(5.0, result, EPSILON);
    Mockito.verify(mockLimiter).calculate(5.0);
  }

  @Test
  public void testLimitThenClamp_limiterReturnsAboveMax() {
    SlewRateLimiter mockLimiter = Mockito.mock(SlewRateLimiter.class);
    Mockito.when(mockLimiter.calculate(8.0)).thenReturn(12.0);

    double result = MathUtils.limitThenClamp(mockLimiter, 8.0, 0.0, 10.0);

    assertEquals(10.0, result, EPSILON);
    Mockito.verify(mockLimiter).calculate(8.0);
  }

  @Test
  public void testLimitThenClamp_limiterReturnsBelowMin() {
    SlewRateLimiter mockLimiter = Mockito.mock(SlewRateLimiter.class);
    Mockito.when(mockLimiter.calculate(2.0)).thenReturn(-1.0);

    double result = MathUtils.limitThenClamp(mockLimiter, 2.0, 0.0, 10.0);

    assertEquals(0.0, result, EPSILON);
    Mockito.verify(mockLimiter).calculate(2.0);
  }
}
