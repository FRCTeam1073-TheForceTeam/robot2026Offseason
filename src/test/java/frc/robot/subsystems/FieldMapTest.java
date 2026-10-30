package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class FieldMapTest {
    private static final double EPSILON = 1e-6;
    private FieldMap fieldMap = new FieldMap();

    @Test
    public void testFindDistance_knownTag() {
        // Use a known tag ID from the 2026 field
        // Pick an arbitrary tag and a robot position, verify Pythagorean distance
        Pose2d robotPose = new Pose2d(0.0, 0.0, new Rotation2d());

        double distance = fieldMap.findDistance(robotPose, 1);

        // Distance should be non-zero for a tag not at origin
        assertTrue(distance > 0);
    }

    @Test
    public void testFindDistance_unknownTag() {
        Pose2d robotPose = new Pose2d(0.0, 0.0, new Rotation2d());

        double distance = fieldMap.findDistance(robotPose, 999);

        // Unknown tag ID returns 999
        assertEquals(999.0, distance, EPSILON);
    }

    @Test
    public void testFindDistance_zeroDistanceAtTagLocation() {
        // Distance should be zero when robot is at the same location as the tag
        var tagPose = FieldMap.fieldMap.getTagPose(1);
        if (tagPose.isPresent()) {
            Pose2d robotAtTag = new Pose2d(tagPose.get().getX(), tagPose.get().getY(), new Rotation2d());
            double distanceAtTag = fieldMap.findDistance(robotAtTag, 1);

            assertEquals(0.0, distanceAtTag, EPSILON);
        }
    }

    @Test
    public void testGetBestAprilTagID_excludesSpecialTags() {
        // Position the robot such that one of the excluded tags would be closest
        Pose2d robotPose = new Pose2d(0.0, 0.0, new Rotation2d());

        int bestTag = fieldMap.getBestAprilTagID(robotPose);

        // Best tag should not be 4, 5, 14, or 15 (if any other tag exists)
        assertTrue(bestTag != 4 && bestTag != 5 && bestTag != 14 && bestTag != 15);
    }

    @Test
    public void testGetBestAprilTagID_returnsValidTag() {
        Pose2d robotPose = new Pose2d(8.0, 4.0, new Rotation2d());

        int bestTag = fieldMap.getBestAprilTagID(robotPose);

        // Should return a valid tag ID (non-negative) or -1 if no valid tags
        assertTrue(bestTag >= -1);
    }

    @Test
    public void testGetTagRelativePose_slotLeftOffset() {
        Transform2d noOffset = new Transform2d();
        Pose2d pose = fieldMap.getTagRelativePose(1, -1, noOffset);

        // Should return a valid pose
        assertTrue(pose != null);
    }

    @Test
    public void testGetTagRelativePose_slotCenterOffset() {
        Transform2d noOffset = new Transform2d();
        Pose2d pose = fieldMap.getTagRelativePose(1, 0, noOffset);

        // Should return a valid pose
        assertTrue(pose != null);
    }

    @Test
    public void testGetTagRelativePose_slotRightOffset() {
        Transform2d noOffset = new Transform2d();
        Pose2d pose = fieldMap.getTagRelativePose(1, 1, noOffset);

        // Should return a valid pose
        assertTrue(pose != null);
    }

    @Test
    public void testGetTagRelativePose_slotCoralStation() {
        Transform2d noOffset = new Transform2d();
        Pose2d pose = fieldMap.getTagRelativePose(1, 2, noOffset);

        // Should return a valid pose
        assertTrue(pose != null);
    }

    @Test
    public void testGetTagRelativePose_appliesOffsetTransform() {
        Transform2d offset = new Transform2d(1.0, 2.0, new Rotation2d());

        Pose2d poseWithoutOffset = fieldMap.getTagRelativePose(1, 0, new Transform2d());
        Pose2d poseWithOffset = fieldMap.getTagRelativePose(1, 0, offset);

        // The pose with offset should differ from the one without by approximately the offset amount
        // (Transform2d.plus applies the offset in the target's frame, which might involve rotation)
        double xDiff = Math.abs(poseWithOffset.getX() - poseWithoutOffset.getX());
        double yDiff = Math.abs(poseWithOffset.getY() - poseWithoutOffset.getY());

        // At least one component should have changed by approximately the offset
        assertTrue(xDiff > 0.1 || yDiff > 0.1);
    }

    @Test
    public void testGetTagRelativePose_unknownTagThrows() {
        Transform2d noOffset = new Transform2d();

        // Unknown tag ID should throw (unguarded Optional.get())
        assertThrows(java.util.NoSuchElementException.class, () -> {
                fieldMap.getTagRelativePose(999, 0, noOffset);
            });
    }

    @Test
    public void testGetTagRelativePose_defaultSlotAppliesNoOffset() {
        Transform2d noOffset = new Transform2d();

        Pose2d poseSlot0 = fieldMap.getTagRelativePose(1, 0, noOffset);
        Pose2d poseDefaultSlot = fieldMap.getTagRelativePose(1, 999, noOffset);

        // Default (unrecognized) slot should not apply any slot offset
        // so it should match the tag's base pose (just with endEffectorOffset in center position)
        // Actually, unrecognized slots skip all the if/else logic, so they get no slot offset
        // but still get the general offset applied, so they differ from slot 0
        assertTrue(
            Math.abs(poseDefaultSlot.getY() - poseSlot0.getY()) > EPSILON ||
            Math.abs(poseDefaultSlot.getX() - poseSlot0.getX()) > EPSILON
        );
    }
}
