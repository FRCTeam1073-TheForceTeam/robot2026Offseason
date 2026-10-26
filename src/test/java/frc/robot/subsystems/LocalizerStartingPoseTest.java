package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests the localizer's starting pose. Nothing seeds the pose from the robot's real starting position, so these tests
 * document the current behavior: it starts at the origin and only moves when reset or updated.
 *
 * These pass today. Seeding the starting pose from the field position is a design choice left to the students.
 */
public class LocalizerStartingPoseTest {
    private static final double TOLERANCE = 1e-9;

    private LocalizerTest.MockDrivetrain drivetrain;
    private Localizer localizer;

    @BeforeEach
    public void setUp() {
        drivetrain = new LocalizerTest.MockDrivetrain();
        localizer = new Localizer(drivetrain, new LocalizerTest.MockAprilTagFinder(new LocalizerTest.MockTurret()));
    }

    @Test
    public void testPoseStartsAtOrigin() {
        Pose2d pose = localizer.getPose();

        assertEquals(0.0, pose.getX(), TOLERANCE, "Localizer should start with x at the origin");
        assertEquals(0.0, pose.getY(), TOLERANCE, "Localizer should start with y at the origin");
    }

    @Test
    public void testPoseDoesNotChangeUntilPeriodic() {
        localizer.resetPose(new Pose2d(3.0, 4.0, new Rotation2d(0.0)));

        assertEquals(0.0, localizer.getPose().getX(), TOLERANCE,
            "getPose() returns the cached pose, so a reset does not show until periodic() runs");
    }

    @Test
    public void testResetPoseTakesEffectOnNextPeriodic() {
        localizer.resetPose(new Pose2d(3.0, 4.0, new Rotation2d(0.0)));

        localizer.periodic();

        assertEquals(3.0, localizer.getPose().getX(), 0.01, "Reset pose x should be used after periodic()");
        assertEquals(4.0, localizer.getPose().getY(), 0.01, "Reset pose y should be used after periodic()");
    }
}
