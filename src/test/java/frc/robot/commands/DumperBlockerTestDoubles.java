package frc.robot.commands;

import frc.robot.subsystems.DumperBlocker;
import frc.robot.subsystems.Intake;

/**
 * Test doubles for the dumper blocker command tests. Each double records what a command asked for or returns
 * values the test sets directly.
 */
final class DumperBlockerTestDoubles {
    private DumperBlockerTestDoubles() {
    }

    /** DumperBlocker double that records position and zeroing commands instead of driving hardware. */
    static class RecordingDumperBlocker extends DumperBlocker {
        double lastSetPosition = Double.NaN;
        boolean stopCalled = false;
        boolean zeroCalled = false;
        boolean setZeroCalled = false;
        double velocityRotorRps = 0.0;
        double position = 0.0;

        @Override
        public void setPosition(double radians) {
            lastSetPosition = radians;
        }

        @Override
        public double getPositionRadians() {
            return position;
        }

        @Override
        public void stop() {
            stopCalled = true;
        }

        @Override
        public void zero() {
            zeroCalled = true;
        }

        @Override
        public void setZero() {
            setZeroCalled = true;
        }

        @Override
        public double getVelocityRotorRps() {
            return velocityRotorRps;
        }
    }

    /** Intake double whose stowed state and position are set directly by each test. */
    static class FakeIntake extends Intake {
        boolean stowed = true;
        double position = Intake.stowedPositionRadians;
        double lastSetPosition = Double.NaN;

        @Override
        public boolean isStowed() {
            return stowed;
        }

        @Override
        public void setPosition(double radians) {
            lastSetPosition = radians;
        }

        @Override
        public void stop() {
        }

        @Override
        public double getPositionRadians() {
            return position;
        }
    }
}
