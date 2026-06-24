// MAP: loads the map

package frc.robot.subsystems;


import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class FieldMap
{
    // TODO: AprilTagFields has two flavors of reefscape map: andymark and welded. which one do you need? who knows!
    // apparently for the 2/15 event it's welded. after that all bets are off.
    public static final AprilTagFieldLayout fieldMap = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    
    public static final HashMap<Integer, Integer> algaeHeight = new HashMap<>() {{
        put(6, 0);
        put(7, 1);
        put(8, 0);
        put(9, 1);
        put(10, 0);
        put(11, 1);
        put(17, 0);
        put(18, 1);
        put(19, 0);
        put(20, 1);
        put(21, 0);
        put(22, 1);
    }};
    
    public int getBestAprilTagID(Pose2d robotPose) 
    {
        double shortestDistance = 998;

        List<AprilTag> aprilTags = fieldMap.getTags();
        int bestID = -1;

        for(AprilTag tag : aprilTags) 
        {
            if (findDistance(robotPose, tag.ID) < shortestDistance && tag.ID != 4 && tag.ID != 5 && tag.ID != 14 && tag.ID != 15) 
            {
                shortestDistance = findDistance(robotPose, tag.ID);
                bestID = tag.ID;
            }
        }
        return bestID;
    }

    public int getBestReefTagID(Pose2d robotPose)
    {
        double shortestDistance = 998;

        List<AprilTag> aprilTags = fieldMap.getTags();
        int bestID = -1;

        for(AprilTag tag : aprilTags) 
        {
            if (findDistance(robotPose, tag.ID) < shortestDistance && ((tag.ID >= 6 && tag.ID <= 11) || (tag.ID >= 17 && tag.ID <= 22))) 
            {
                shortestDistance = findDistance(robotPose, tag.ID);
                bestID = tag.ID;
            }
        }
        SmartDashboard.putNumber("FieldMap/", bestID);
        return bestID;
    }

    public int getBestSourceTagID(Pose2d robotPose, boolean isRed)
    {
        double shortestDistance = 998;

        List<AprilTag> aprilTags = fieldMap.getTags();
        int bestID = -1;

        for(AprilTag tag : aprilTags) 
        {
            if (isRed)
            {
                if (findDistance(robotPose, tag.ID) < shortestDistance && (tag.ID <= 2 && tag.ID > 0)) 
                {
                    shortestDistance = findDistance(robotPose, tag.ID);
                    bestID = tag.ID;
                }
            }
            else
            {
                if (findDistance(robotPose, tag.ID) < shortestDistance && (tag.ID == 12 || tag.ID == 13)) 
                {
                    shortestDistance = findDistance(robotPose, tag.ID);
                    bestID = tag.ID;
                }
            }
        }
        return bestID;
    }

    public Pose2d getTagRelativePose(int tagID, int slot, Transform2d offset)
    {
        Pose2d tagPose = fieldMap.getTagPose(tagID).get().toPose2d();
        double yOffset = 0.165;
        // double endEffectorOffset = 0.2286;
        double endEffectorOffset = 0.1905;

        if(slot == -1) // left
        {
            tagPose = tagPose.plus(new Transform2d(0, -yOffset + endEffectorOffset, new Rotation2d()));
        }
        else if(slot == 0) // center
        {
            tagPose = tagPose.plus(new Transform2d(0, endEffectorOffset, new Rotation2d()));
        }
        else if(slot == 1) // right
        {
            tagPose = tagPose.plus(new Transform2d(0, yOffset + endEffectorOffset, new Rotation2d()));
        }
        else if (slot == 2) // coral station
        {
            tagPose = tagPose.plus(new Transform2d(0, -endEffectorOffset, new Rotation2d()));
        }

        tagPose = tagPose.plus(offset);

        SmartDashboard.putNumber("FieldMap/TargetTagID", tagID);
        SmartDashboard.putNumber("FieldMap/TargetPoseX", tagPose.getX());
        SmartDashboard.putNumber("FieldMap/TargetPoseY", tagPose.getY());
        SmartDashboard.putNumber("FieldMap/TargetPoseTheta", tagPose.getRotation().getRadians());

        return tagPose;
    }

    public double findDistance(Pose2d robot2DPose, int tagID) {

        Optional<Pose3d> tag3dPose = FieldMap.fieldMap.getTagPose(tagID);

        if(!tag3dPose.isPresent()) {
            //we don't have a value to work with, bail
            return 999;
        }

        double tagX = tag3dPose.get().getX();
        double tagY = tag3dPose.get().getY();
        double distance = Math.sqrt(Math.pow((tagX - robot2DPose.getX()), 2) + Math.pow((tagY - robot2DPose.getY()), 2));
        return distance;
    }
}
