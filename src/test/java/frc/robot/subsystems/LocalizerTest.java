package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import frc.robot.subsystems.AprilTagFinder.VisionMeasurement;

public class LocalizerTest {
    private Localizer localizer;
    private MockDrivetrain mockDrivetrain;
    private MockAprilTagFinder mockFinder;
    private MockTurret mockTurret;

    @BeforeEach
    public void setUp() {
        mockDrivetrain = new MockDrivetrain();
        mockTurret = new MockTurret();
        mockFinder = new MockAprilTagFinder(mockTurret, mockDrivetrain);
        localizer = new Localizer(mockDrivetrain, mockFinder);
    }

    @Test
    public void testMeasurementStableWhenRobotStopped() {
        mockDrivetrain.setSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
        localizer.setLinearSpeedThreshold(2.5);
        localizer.setAngularSpeedThreshold(2.0);

        localizer.periodic();

        assertTrue(mockDrivetrain.getWasAddVisionMeasurementCalled() == false,
            "Measurement should not be added on first periodic call");
    }

    @Test
    public void testMeasurementStableFailsWhenLinearSpeedExceedsThreshold() {
        mockDrivetrain.setSpeeds(new ChassisSpeeds(3.0, 0.0, 0.0));
        localizer.setLinearSpeedThreshold(2.5);
        localizer.setAngularSpeedThreshold(2.0);

        localizer.periodic();

        assertFalse(mockDrivetrain.getWasAddVisionMeasurementCalled(),
            "Should not add vision measurement when linear speed exceeds threshold");
    }

    @Test
    public void testMeasurementStableFailsWhenAngularSpeedExceedsThreshold() {
        mockDrivetrain.setSpeeds(new ChassisSpeeds(0.0, 0.0, 3.0));
        localizer.setLinearSpeedThreshold(2.5);
        localizer.setAngularSpeedThreshold(2.0);

        localizer.periodic();

        assertFalse(mockDrivetrain.getWasAddVisionMeasurementCalled(),
            "Should not add vision measurement when angular speed exceeds threshold");
    }

    @Test
    public void testProcessVisionMeasurementsWithNoMeasurements() {
        mockDrivetrain.setSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
        mockFinder.setMeasurements(new ArrayList<>());

        localizer.periodic();
        localizer.periodic();

        assertFalse(mockDrivetrain.getWasAddVisionMeasurementCalled(),
            "Should not call addVisionMeasurement when no measurements available");
    }

    @Test
    public void testProcessVisionMeasurementsWithValidMeasurements() {
        mockDrivetrain.setSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
        localizer.setTimeGap(0.0);
        List<VisionMeasurement> measurements = new ArrayList<>();
        measurements.add(new VisionMeasurement(
            new Pose2d(1.0, 2.0, new Rotation2d(0.0)),
            new Transform2d(),
            Timer.getFPGATimestamp(),
            1,
            new double[]{0.1, 0.1, 0.1}
        ));
        mockFinder.setMeasurements(measurements);

        localizer.periodic();

        assertTrue(mockDrivetrain.getWasAddVisionMeasurementCalled(),
            "Should call addVisionMeasurement when measurements are available and robot is stable");
    }

    @Test
    public void testProcessVisionMeasurementsClearsAfterProcessing() {
        mockDrivetrain.setSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
        localizer.setTimeGap(0.0);
        List<VisionMeasurement> measurements = new ArrayList<>();
        measurements.add(new VisionMeasurement(
            new Pose2d(1.0, 2.0, new Rotation2d(0.0)),
            new Transform2d(),
            Timer.getFPGATimestamp(),
            1,
            new double[]{0.1, 0.1, 0.1}
        ));
        mockFinder.setMeasurements(measurements);

        localizer.periodic();

        assertTrue(mockFinder.getWasClearMeasurementsCalled(),
            "Should clear measurements after processing");
    }

    @Test
    public void testLinearSpeedCalculation() {
        mockDrivetrain.setSpeeds(new ChassisSpeeds(3.0, 4.0, 0.0));
        localizer.setLinearSpeedThreshold(5.5);

        localizer.periodic();

        assertFalse(mockDrivetrain.getWasAddVisionMeasurementCalled(),
            "Speed should be sqrt(3^2 + 4^2) = 5.0, which should be stable at 5.5 threshold");
    }

    static class MockDrivetrain extends Drivetrain {
        private ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        private boolean wasAddVisionMeasurementCalled = false;

        public MockDrivetrain() {
            super();
        }

        @Override
        public ChassisSpeeds getChassisSpeeds() {
            return speeds;
        }

        @Override
        public Pose2d getOdometry() {
            return new Pose2d();
        }

        @Override
        public void addVisionMeasurement(Pose2d visionRobotPose, double timestamp, double[] stddevs) {
            wasAddVisionMeasurementCalled = true;
        }

        @Override
        public void resetOdometry(Pose2d newPose) {
        }

        @Override
        public void resetRotation(Rotation2d rotation) {
        }

        public void setSpeeds(ChassisSpeeds newSpeeds) {
            speeds = newSpeeds;
        }

        public boolean getWasAddVisionMeasurementCalled() {
            return wasAddVisionMeasurementCalled;
        }
    }

    static class MockAprilTagFinder extends AprilTagFinder {
        private List<VisionMeasurement> measurements = new ArrayList<>();
        private boolean wasClearMeasurementsCalled = false;

        public MockAprilTagFinder(Turret turret, Drivetrain drivetrain) {
            super(turret, drivetrain);
        }

        @Override
        public List<VisionMeasurement> getAllMeasurements() {
            return new ArrayList<>(measurements);
        }

        @Override
        public void clearMeasurements() {
            wasClearMeasurementsCalled = true;
            measurements.clear();
        }

        public void setMeasurements(List<VisionMeasurement> newMeasurements) {
            measurements = newMeasurements;
            wasClearMeasurementsCalled = false;
        }

        public boolean getWasClearMeasurementsCalled() {
            return wasClearMeasurementsCalled;
        }
    }

    static class MockTurret extends Turret {
        public MockTurret() {
            super();
        }
    }
}
