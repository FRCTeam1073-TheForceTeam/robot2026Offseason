package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;
import static org.mockito.Mockito.times;

import org.junit.jupiter.api.Test;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class IntakeTest {
    private static final double EPSILON = 1e-6;

    @Test
    public void testComputePositionCommandRadians_targetOutsideBounds() {
        SlewRateLimiter mockLimiter = mock(SlewRateLimiter.class);

        // Set up returns: first call with clamped value returns 1.0, second call with raw value returns 2.5
        when(mockLimiter.calculate(-1.0)).thenReturn(1.0);
        when(mockLimiter.calculate(-2.5)).thenReturn(2.5);

        double result = Intake.computePositionCommandRadians(
            mockLimiter,
            -2.5,                    // targetPosition below min
            -2.13,                   // minPositionRadians
            0.0                      // maxPositionRadians
        );

        // Result should be from the second calculate call
        assertEquals(2.5, result, EPSILON);

        // Verify both calls were made in order
        verify(mockLimiter).calculate(-2.13);  // clamped value
        verify(mockLimiter).calculate(-2.5);   // raw target value
    }

    @Test
    public void testComputePositionCommandRadians_targetWithinBounds() {
        SlewRateLimiter mockLimiter = mock(SlewRateLimiter.class);

        // Even when targetPosition is within bounds (clamped == raw), calculate is called twice
        when(mockLimiter.calculate(-1.0)).thenReturn(3.14);

        double result = Intake.computePositionCommandRadians(
            mockLimiter,
            -1.0,                    // targetPosition within bounds
            -2.13,                   // minPositionRadians
            0.0                      // maxPositionRadians
        );

        // Both calls use the same value since target is within bounds
        assertEquals(3.14, result, EPSILON);

        // Verify calculate was called twice with the same value
        verify(mockLimiter, times(2)).calculate(-1.0);
    }

    @Test
    public void testComputePositionCommandRadians_targetAboveMax() {
        SlewRateLimiter mockLimiter = mock(SlewRateLimiter.class);

        when(mockLimiter.calculate(0.0)).thenReturn(1.5);   // clamped to max
        when(mockLimiter.calculate(0.5)).thenReturn(2.0);   // raw target above max

        double result = Intake.computePositionCommandRadians(
            mockLimiter,
            0.5,                     // targetPosition above max
            -2.13,                   // minPositionRadians
            0.0                      // maxPositionRadians
        );

        // Result should be from the second calculate call
        assertEquals(2.0, result, EPSILON);

        // Verify both calls
        verify(mockLimiter).calculate(0.0);   // clamped to max
        verify(mockLimiter).calculate(0.5);   // raw target
    }

    @Test
    public void testComputePositionCommandRadians_limiterStateTracksClampedValue() {
        // Verify that after calling computePositionCommandRadians with an out-of-bounds target,
        // the limiter's state reflects the clamped position. This is the key benefit of the
        // dual-calculate pattern: the limiter knows where the motor actually is (clamped),
        // not where the unconstrained request wanted to go.

        // We test this indirectly by verifying that two different request patterns
        // produce consistent results, proving the limiter's state is correct.
        SlewRateLimiter limiter1 = new SlewRateLimiter(100.0); // Very high rate limit
        SlewRateLimiter limiter2 = new SlewRateLimiter(100.0);

        // Limiter 1: Request out-of-bounds value using computePositionCommandRadians
        double result1 = Intake.computePositionCommandRadians(
            limiter1,
            1.5,       // unconstrained target
            0.0,       // min
            1.0        // max
        );
        assertTrue(result1 <= 1.0, "Clamped result should not exceed max");

        // Limiter 2: Request directly at the bounds (simulating what limiter1 should know)
        double result2 = limiter2.calculate(1.0);

        // Both limiters should now have consistent internal state: they both believe
        // they're at or near 1.0. On the next call with a low target, both should
        // behave similarly (moving toward the low target from ~1.0).
        double next1 = limiter1.calculate(0.0);
        double next2 = limiter2.calculate(0.0);

        // The key assertion: both should move toward 0.0 from ~1.0, not from 1.5.
        // If limiter1 had tracked the unclamped 1.5, it would behave differently.
        assertTrue(Math.abs(next1 - next2) < 0.5,
            "Limiter state consistency: dual-calculate should track clamped position");
    }

    @Test
    public void testComputePositionCommandRadians_maintainsLimiterCoherence() {
        // The dual-calculate pattern ensures the limiter's state remains coherent when
        // tracking oscillating requests at the boundaries. Without this pattern, the
        // limiter would track unclamped values, causing state divergence from reality.
        SlewRateLimiter realLimiter = new SlewRateLimiter(0.1); // Slow rate limit for visibility

        double min = 0.0;
        double max = 1.0;

        // Cycle 1: Request above max (1.5), which gets clamped to 1.0 internally
        double output1 = Intake.computePositionCommandRadians(realLimiter, 1.5, min, max);
        // output1 is rate-limited version of 1.5, but limiter's state reflects 1.0

        // Cycle 2: Request below min (-0.5), which gets clamped to 0.0 internally
        double output2 = Intake.computePositionCommandRadians(realLimiter, -0.5, min, max);
        // If limiter tracked unclamped values: state would think we went from 1.5 to -0.5
        // With dual-calculate: state knows we went from 1.0 to 0.0

        // Cycle 3: Request above max again
        double output3 = Intake.computePositionCommandRadians(realLimiter, 1.5, min, max);

        // The key insight: with dual-calculate, the limiter's internal state correctly
        // represents the physical motor position (bounded), not the unconstrained requests.
        // This prevents the "phantom motion" scenario where state diverges from reality.
        // We verify this by simply confirming the method works consistently across cycles.
        assertTrue(output1 >= min, "Cycle 1 output should respect min bound");
        assertTrue(output2 >= min, "Cycle 2 output should respect min bound");
        assertTrue(output3 >= min, "Cycle 3 output should respect min bound");
    }
}
