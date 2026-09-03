package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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
        mockFinder = new MockAprilTagFinder(mockTurret);
        localizer = new Localizer(mockDrivetrain, mockFinder);
    }

    @Test
    public void testMeasurementStableWhenRobotStopped() {
        mockDrivetrain.setSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
        localizer.setLinearSpeedThreshold(2.5);
        localizer.setAngularSpeedThreshold(2.0);

        localizer.periodic();

        // Pose should remain at origin when no measurements are added
        assertEquals(0.0, localizer.getPose().getX(), 0.01,
            "Pose should not change on first periodic call without measurements");
    }

    @Test
    public void testMeasurementStableFailsWhenLinearSpeedExceedsThreshold() {
        mockDrivetrain.setSpeeds(new ChassisSpeeds(3.0, 0.0, 0.0));
        localizer.setLinearSpeedThreshold(2.5);
        localizer.setAngularSpeedThreshold(2.0);

        Pose2d initialPose = localizer.getPose();
        localizer.setTimeGap(0.0);
        List<VisionMeasurement> measurements = new ArrayList<>();
        measurements.add(new VisionMeasurement(
            new Pose2d(5.0, 5.0, new Rotation2d(0.0)),
            new Transform2d(),
            Timer.getFPGATimestamp(),
            1,
            new double[]{0.1, 0.1, 0.1}
        ));
        mockFinder.setMeasurements(measurements);

        localizer.periodic();

        assertEquals(initialPose.getX(), localizer.getPose().getX(), 0.01,
            "Should not apply vision measurement when linear speed exceeds threshold");
    }

    @Test
    public void testMeasurementStableFailsWhenAngularSpeedExceedsThreshold() {
        mockDrivetrain.setSpeeds(new ChassisSpeeds(0.0, 0.0, 3.0));
        localizer.setLinearSpeedThreshold(2.5);
        localizer.setAngularSpeedThreshold(2.0);

        Pose2d initialPose = localizer.getPose();
        localizer.setTimeGap(0.0);
        List<VisionMeasurement> measurements = new ArrayList<>();
        measurements.add(new VisionMeasurement(
            new Pose2d(5.0, 5.0, new Rotation2d(0.0)),
            new Transform2d(),
            Timer.getFPGATimestamp(),
            1,
            new double[]{0.1, 0.1, 0.1}
        ));
        mockFinder.setMeasurements(measurements);

        localizer.periodic();

        assertEquals(initialPose.getX(), localizer.getPose().getX(), 0.01,
            "Should not apply vision measurement when angular speed exceeds threshold");
    }

    @Test
    public void testProcessVisionMeasurementsWithNoMeasurements() {
        mockDrivetrain.setSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
        mockFinder.setMeasurements(new ArrayList<>());

        localizer.periodic();
        Pose2d poseAfterFirstPeriodic = localizer.getPose();

        localizer.periodic();
        Pose2d poseAfterSecondPeriodic = localizer.getPose();

        assertEquals(poseAfterFirstPeriodic.getX(), poseAfterSecondPeriodic.getX(), 0.01,
            "Pose should not change when no measurements available");
    }

    @Test
    public void testVisionMeasurementsUpdatePoseEstimator() {
        mockDrivetrain.setSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
        localizer.setTimeGap(0.0);

        Pose2d visionPose = new Pose2d(3.0, 4.0, new Rotation2d(0.0));
        List<VisionMeasurement> measurements = new ArrayList<>();
        measurements.add(new VisionMeasurement(
            visionPose,
            new Transform2d(),
            Timer.getFPGATimestamp(),
            1,
            new double[]{0.1, 0.1, 0.1}
        ));
        mockFinder.setMeasurements(measurements);

        localizer.periodic();

        // Pose estimator should move towards the vision measurement
        assertTrue(localizer.getPose().getX() > 0.0,
            "Pose estimator should be updated with vision measurement");
        assertTrue(localizer.getPose().getY() > 0.0,
            "Pose estimator Y should be updated with vision measurement");
    }

    @Test
    public void testMultipleVisionMeasurementsUpdatePose() {
        mockDrivetrain.setSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
        localizer.setTimeGap(0.0);

        Pose2d initialPose = localizer.getPose();

        // First measurement
        List<VisionMeasurement> measurements1 = new ArrayList<>();
        measurements1.add(new VisionMeasurement(
            new Pose2d(2.0, 2.0, new Rotation2d(0.0)),
            new Transform2d(),
            Timer.getFPGATimestamp(),
            1,
            new double[]{0.1, 0.1, 0.1}
        ));
        mockFinder.setMeasurements(measurements1);
        localizer.periodic();

        Pose2d poseAfterFirstMeasurement = localizer.getPose();
        assertTrue(poseAfterFirstMeasurement.getX() > initialPose.getX(),
            "First vision measurement should move pose");

        // Second measurement at different location
        localizer.setTimeGap(0.0);
        List<VisionMeasurement> measurements2 = new ArrayList<>();
        measurements2.add(new VisionMeasurement(
            new Pose2d(5.0, 5.0, new Rotation2d(0.0)),
            new Transform2d(),
            Timer.getFPGATimestamp(),
            1,
            new double[]{0.1, 0.1, 0.1}
        ));
        mockFinder.setMeasurements(measurements2);
        localizer.periodic();

        Pose2d poseAfterSecondMeasurement = localizer.getPose();
        assertTrue(poseAfterSecondMeasurement.getX() > poseAfterFirstMeasurement.getX(),
            "Second vision measurement should move pose further");
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
        // Speed = sqrt(3^2 + 4^2) = 5.0 m/s, threshold = 5.5 m/s, so should be stable
        mockDrivetrain.setSpeeds(new ChassisSpeeds(3.0, 4.0, 0.0));
        localizer.setLinearSpeedThreshold(5.5);

        Pose2d initialPose = localizer.getPose();
        localizer.setTimeGap(0.0);
        List<VisionMeasurement> measurements = new ArrayList<>();
        measurements.add(new VisionMeasurement(
            new Pose2d(5.0, 5.0, new Rotation2d(0.0)),
            new Transform2d(),
            Timer.getFPGATimestamp(),
            1,
            new double[]{0.1, 0.1, 0.1}
        ));
        mockFinder.setMeasurements(measurements);

        localizer.periodic();

        // Vision should apply because 5.0 m/s <= 5.5 m/s threshold
        assertTrue(localizer.getPose().getX() > initialPose.getX(),
            "Speed should be sqrt(3^2 + 4^2) = 5.0, which is stable at 5.5 threshold, should apply vision");
    }

    static class MockDrivetrain extends Drivetrain {
        private ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        private Pose2d odometry = new Pose2d();
        private Rotation2d gyroHeading = new Rotation2d(0.0);

        public MockDrivetrain() {
            super();
        }

        @Override
        public ChassisSpeeds getChassisSpeeds() {
            return speeds;
        }

        @Override
        public Pose2d getOdometry() {
            return odometry;
        }

        @Override
        public Rotation2d getGyroHeading() {
            return gyroHeading;
        }

        @Override
        public SwerveModulePosition[] getSwerveModulePositions() {
            return new SwerveModulePosition[]{
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };
        }

        @Override
        public void addVisionMeasurement(Pose2d visionRobotPose, double timestamp, double[] stddevs) {
        }

        @Override
        public void resetOdometry(Pose2d newPose) {
            odometry = newPose;
        }

        @Override
        public void resetRotation(Rotation2d rotation) {
            gyroHeading = rotation;
        }

        public void setSpeeds(ChassisSpeeds newSpeeds) {
            speeds = newSpeeds;
        }
    }

    static class MockAprilTagFinder extends AprilTagFinder {
        private List<VisionMeasurement> measurements = new ArrayList<>();
        private boolean wasClearMeasurementsCalled = false;

        public MockAprilTagFinder(Turret turret) {
            super(turret);
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
