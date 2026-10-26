package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

/**
 * Sanity checks on the DumperBlocker tuning constants. The arm's motion and gravity compensation run through the
 * TalonFX, so these checks cover the values the rest of the code depends on rather than the hardware behavior.
 */
public class DumperBlockerTest {

    @Test
    public void testDeployedIsBeyondStowed() {
        assertTrue(DumperBlocker.deployedPosition > DumperBlocker.stowedPosition,
            "Deployed setpoint should be further out than the stowed setpoint");
    }

    @Test
    public void testSetpointsWithinMotionRange() {
        assertTrue(DumperBlocker.stowedPosition >= DumperBlocker.minPositionRadians,
            "Stowed setpoint should be inside the commanded range");
        assertTrue(DumperBlocker.deployedPosition <= DumperBlocker.maxPositionRadians,
            "Deployed setpoint should be inside the commanded range");
    }

    @Test
    public void testExtendAndRetractVelocitiesMirror() {
        assertEquals(DumperBlocker.extendVelocity, -DumperBlocker.retractVelocity, 1e-9,
            "Extend and retract should drive at equal speeds in opposite directions");
    }

    @Test
    public void testZeroingDrivesTowardStowedPosition() {
        assertTrue(DumperBlocker.zeroVelocity < 0.0,
            "Zeroing should drive the arm down, away from the deployed position");
        assertEquals(0.0, DumperBlocker.stowedPosition, 1e-9,
            "Zeroing resets the encoder to 0, so the stowed position must be 0 for the reset to match the arm");
    }

    @Test
    public void testZeroDrivesFasterThanStallThreshold() {
        assertTrue(Math.abs(DumperBlocker.zeroVelocity) > DumperBlocker.zeroStallVelocity,
            "Zeroing speed must be above the stall threshold, or the arm would look stalled while moving");
    }

    @Test
    public void testStallTimingsPositive() {
        assertTrue(DumperBlocker.zeroStartupSeconds > 0.0, "Startup grace period should be positive");
        assertTrue(DumperBlocker.zeroStallSeconds > 0.0, "Stall confirmation time should be positive");
    }

    @Test
    public void testMotionProfileLimitsPositive() {
        assertTrue(DumperBlocker.cruiseVelocity > 0.0, "Cruise velocity should be positive");
        assertTrue(DumperBlocker.acceleration > 0.0, "Acceleration should be positive");
    }

    @Test
    public void testArrivedToleranceSmallButPositive() {
        assertTrue(DumperBlocker.arrivedTolerance > 0.0, "Arrival tolerance should be positive");
        assertTrue(DumperBlocker.arrivedTolerance < DumperBlocker.deployedPosition - DumperBlocker.stowedPosition,
            "Arrival tolerance should be smaller than the full travel between stowed and deployed");
    }

    @Test
    public void testGravityFeedforwardPositive() {
        assertTrue(DumperBlocker.gravityFeedforwardVolts > 0.0,
            "Gravity compensation should push against gravity, so it must be positive");
    }

    @Test
    public void testCurrentLimitsPositive() {
        assertTrue(DumperBlocker.currentLimit > 0.0, "Supply current limit should be positive");
        assertTrue(DumperBlocker.hardstopCurrent > 0.0, "Hardstop current should be positive");
    }
}
