package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.commands.DrivePath;
import frc.robot.utilities.TrajectoryWarmup;

/** Checks the real .traj files that the robot ships with. */
public class TrajectoryContentTest {
    static Iterable<String> trajectoryFiles() {
        return AutoCatalog.trajectoryFiles();
    }

    @Test
    public void testEveryTrajectoryFileExistsOnDisk() {
        for (String name : AutoCatalog.trajectoryFiles()) {
            Path file = Path.of("src/main/deploy/choreo", name + ".traj");
            assertTrue(Files.isRegularFile(file), "missing trajectory file: " + file);
        }
    }

    @ParameterizedTest
    @MethodSource("trajectoryFiles")
    public void testTrajectoryLoadsAndHasDuration(String name) {
        Trajectory<SwerveSample> traj = load(name);
        assertTrue(traj.getTotalTime() > 0.0, name + " has no duration");
    }

    @ParameterizedTest
    @MethodSource("trajectoryFiles")
    public void testTrajectoryHasStartPoseForBothAlliances(String name) {
        Trajectory<SwerveSample> traj = load(name);
        assertTrue(traj.getInitialPose(false).isPresent(), name + " has no blue start pose");
        assertTrue(traj.getInitialPose(true).isPresent(), name + " has no red start pose");
    }

    @ParameterizedTest
    @MethodSource("trajectoryFiles")
    public void testSamplesAreFinite(String name) {
        Trajectory<SwerveSample> traj = load(name);
        double total = traj.getTotalTime();
        int steps = 100;

        for (int i = 0; i <= steps; i++) {
            Optional<SwerveSample> sample = traj.sampleAt(total * i / steps, false);
            assertTrue(sample.isPresent(), name + " has no sample at step " + i);

            SwerveSample s = sample.get();
            assertTrue(Double.isFinite(s.x) && Double.isFinite(s.y) && Double.isFinite(s.heading), name + " has a non-finite pose");
        }
    }

    /**
     * Reports, but does not fail on, paths whose feedforward speed is above DrivePath's cap.
     * DrivePath clamps the combined command, so these paths run but are not followed exactly.
     */
    @ParameterizedTest
    @MethodSource("trajectoryFiles")
    public void reportFeedforwardAboveSpeedCap(String name) {
        Trajectory<SwerveSample> traj = load(name);
        double total = traj.getTotalTime();
        int steps = 100;
        double peak = 0.0;

        for (int i = 0; i <= steps; i++) {
            SwerveSample s = traj.sampleAt(total * i / steps, false).orElseThrow();
            // Same feedforward DrivePath adds to its feedback, so this is the speed the robot is commanded.
            peak = Math.max(peak, Math.hypot(s.getChassisSpeeds().vxMetersPerSecond, s.getChassisSpeeds().vyMetersPerSecond));
        }

        if (peak > DrivePath.MAX_TRANSLATION_SPEED) {
            System.err.println("WARNING " + name + " peak feedforward " + peak + " m/s is above the "
                    + DrivePath.MAX_TRANSLATION_SPEED + " m/s DrivePath cap");
        }
    }

    @ParameterizedTest
    @MethodSource("trajectoryFiles")
    public void testWarmupRunsWithoutThrowing(String name) {
        // The warmup runs on a background thread at runtime, so it must never throw.
        TrajectoryWarmup.touchSamples(load(name));
    }

    private static Trajectory<SwerveSample> load(String name) {
        Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory(name);
        assertTrue(traj.isPresent(), "Choreo could not load " + name);
        return traj.get();
    }
}
