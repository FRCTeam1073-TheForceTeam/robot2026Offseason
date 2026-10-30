package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.mockito.Mockito;

public class BallisticShotTest {
    private static final double EPSILON = 1e-5;

    @Test
    public void testComputeShot_midRange() {
        BallisticShot.Shot shot = BallisticShot.computeShot(2.0);

        // Exact values computed from the ballistic formula
        assertEquals(8.896471, shot.flywheelSpeed, EPSILON);
        assertEquals(1.165082, shot.hoodAngle, EPSILON);
        assertEquals(1.155234, shot.shotTime, EPSILON);
    }

    @Test
    public void testComputeShot_justBelowShortShotCutoff() {
        BallisticShot.Shot shot = BallisticShot.computeShot(1.4999);

        // Below 1.5, so velocity is downscaled
        assertEquals(8.341517, shot.flywheelSpeed, EPSILON);
        assertEquals(1.224894, shot.hoodAngle, EPSILON);
        assertEquals(1.155234, shot.shotTime, EPSILON);
    }

    @Test
    public void testComputeShot_atShortShotCutoff() {
        BallisticShot.Shot shot = BallisticShot.computeShot(1.5);

        // At exactly 1.5, strict < means no downscale
        assertEquals(8.780565, shot.flywheelSpeed, EPSILON);
        assertEquals(1.224882, shot.hoodAngle, EPSILON);
        assertEquals(1.155234, shot.shotTime, EPSILON);
    }

    @Test
    public void testComputeShot_atTallShotCutoff() {
        BallisticShot.Shot shot = BallisticShot.computeShot(3.0);

        // At exactly 3.0, strict > means no height scaling yet
        assertEquals(9.219607, shot.flywheelSpeed, EPSILON);
        assertEquals(1.050997, shot.hoodAngle, EPSILON);
        assertEquals(1.155234, shot.shotTime, EPSILON);
    }

    @Test
    public void testComputeShot_justAboveTallShotCutoff() {
        BallisticShot.Shot shot = BallisticShot.computeShot(3.0001);

        // Just over 3.0, height-above-hub starts scaling
        assertEquals(9.219710, shot.flywheelSpeed, EPSILON);
        assertEquals(1.050995, shot.hoodAngle, EPSILON);
        assertEquals(1.155253, shot.shotTime, 0.00001);
    }

    @Test
    public void testComputeShot_largeRange() {
        BallisticShot.Shot shot = BallisticShot.computeShot(5.0);

        assertEquals(11.094283, shot.flywheelSpeed, EPSILON);
        assertEquals(1.018813, shot.hoodAngle, EPSILON);
        assertEquals(1.474662, shot.shotTime, EPSILON);
    }

    @Test
    public void testGetShot_beforePeriodic_returnsConstructorDefault() {
        TargetFinder mockTargetFinder = Mockito.mock(TargetFinder.class);
        BallisticShot ballisticShot = new BallisticShot(mockTargetFinder);

        BallisticShot.Shot defaultShot = ballisticShot.getShot();

        assertEquals(0.0, defaultShot.flywheelSpeed, EPSILON);
        assertEquals(Math.toRadians(69.2), defaultShot.hoodAngle, EPSILON);
        assertEquals(2.0, defaultShot.shotTime, EPSILON);
    }
}
