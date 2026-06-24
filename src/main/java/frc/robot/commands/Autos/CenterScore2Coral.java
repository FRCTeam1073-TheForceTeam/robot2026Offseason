package frc.robot.commands.Autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CoralElevatorToHeight;
import frc.robot.commands.CreepToReef;
import frc.robot.commands.DrivePath;
import frc.robot.commands.LidarAlign;
import frc.robot.commands.LoadCoral;
import frc.robot.commands.Path;
import frc.robot.commands.Path.Point;
import frc.robot.commands.Path.Segment;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.ZeroElevator;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.CoralEndeffector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.Lidar;
import frc.robot.subsystems.Localizer;

public class CenterScore2Coral 
{
    public static Command create(boolean isRed, Drivetrain drivetrain, FieldMap map, Localizer localizer, CoralEndeffector endEffector, CoralElevator elevator, Lidar lidar, int branchLevel)  
    {
        int slot;
        if (branchLevel == 1)
        {
            slot = 0;
        }
        else
        {
            slot = -1;
        }
        Pose2d tag10Pose = map.getTagRelativePose(10, slot, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d tag21Pose = map.getTagRelativePose(21, slot, new Transform2d(AutoConstants.scoreOffsetX, -0.05, new Rotation2d(Math.PI))); // -0.05 y offset is specific to week 2

        Pose2d backPose10 = map.getTagRelativePose(10, slot, new Transform2d(0.5, -0.05, new Rotation2d(Math.PI)));
        Pose2d backPose21 = map.getTagRelativePose(21, slot, new Transform2d(0.5, -0.05, new Rotation2d(Math.PI)));

        Pose2d wall10 = map.getTagRelativePose(10, slot, new Transform2d(0.5, -1, new Rotation2d(Math.PI)));
        Pose2d wall21 = map.getTagRelativePose(21, slot, new Transform2d(0.5, -1, new Rotation2d(Math.PI)));

        Pose2d tag1Pose = map.getTagRelativePose(1, 0, new Transform2d(AutoConstants.loadOffsetX, 0, new Rotation2d()));
        Pose2d tag13Pose = map.getTagRelativePose(13, 0, new Transform2d(AutoConstants.loadOffsetX, 0, new Rotation2d()));

        Pose2d tag6Pose = map.getTagRelativePose(6, 0, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d()));
        Pose2d tag19Pose = map.getTagRelativePose(13, 0, new Transform2d(AutoConstants.scoreOffsetX, -0.05, new Rotation2d()));


        Point start = new Point(localizer.getPose().getX(), localizer.getPose().getY());
        Point tag10 = new Point(tag10Pose.getX(), tag10Pose.getY());
        tag10.blend_radius = AutoConstants.blendRadius;
        Point tag21 = new Point(tag21Pose.getX(), tag21Pose.getY());
        tag21.blend_radius = AutoConstants.blendRadius;

        Point back10 = new Point(backPose10.getX(), backPose10.getY());
        back10.blend_radius = AutoConstants.blendRadius;
        Point back21 = new Point(backPose21.getX(), backPose21.getY());
        back21.blend_radius = AutoConstants.blendRadius;

        Point wall10Point = new Point(wall10.getX(), wall10.getY());
        wall10Point.blend_radius = AutoConstants.blendRadius;
        Point wall21Point = new Point(wall21.getX(), wall21.getY());
        wall21Point.blend_radius = AutoConstants.blendRadius;

        Point tag1 = new Point(tag1Pose.getX(), tag1Pose.getY());
        tag1.blend_radius = AutoConstants.blendRadius;
        Point tag13 = new Point(tag13Pose.getX(), tag13Pose.getY());
        tag13.blend_radius = AutoConstants.blendRadius;

        Point tag6 = new Point(tag6Pose.getX(), tag6Pose.getY());
        tag6.blend_radius = AutoConstants.blendRadius;
        Point tag19 = new Point(tag19Pose.getX(), tag19Pose.getY());
        tag19.blend_radius = AutoConstants.blendRadius;

        ArrayList<Segment> segments = new ArrayList<Segment>();
        ArrayList<Segment> after = new ArrayList<Segment>();
        ArrayList<Segment> toOther = new ArrayList<Segment>();

        Path path;
        Path path2;
        Path path3;
        if (isRed)
        {
            segments.add(new Segment(start, tag10, tag10Pose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));
            after.add(new Segment(tag10, back10, backPose10.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));
            after.add(new Segment (back10, wall10Point, wall10.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));
            after.add(new Segment(wall10Point, tag1, tag1Pose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));
            toOther.add(new Segment(tag1, tag6, tag6Pose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));
            path = new Path(segments, tag10Pose.getRotation().getRadians());
            path2 = new Path(after, tag1Pose.getRotation().getRadians());
            path3 = new Path(toOther, tag6Pose.getRotation().getRadians());
        }
        else
        {
            segments.add(new Segment(start, tag21, tag21Pose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));
            after.add(new Segment(tag21, back21, backPose21.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));
            after.add(new Segment (back21, wall21Point, wall21.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));
            after.add(new Segment(wall21Point, tag13, tag13Pose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));
            toOther.add(new Segment(tag13, tag19, tag19Pose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));
            path = new Path(segments, tag21Pose.getRotation().getRadians());
            path2 = new Path(after, tag13Pose.getRotation().getRadians());
            path3 = new Path(toOther, tag13Pose.getRotation().getRadians());
        }
        

        return new SequentialCommandGroup(
            // TODO: Consider parallel load and drive.
            // new LoadCoral(endEffector),
            // new DrivePath(drivetrain, path, localizer),
            new ParallelCommandGroup(
                new LoadCoral(endEffector),
                new DrivePath(drivetrain, path, localizer)
            ),
            // new LidarAlign(lidar, drivetrain),
            new CoralElevatorToHeight(elevator, branchLevel, true),
            new ParallelRaceGroup( new CoralElevatorToHeight(elevator, branchLevel, false),
                                   new SequentialCommandGroup(new ScoreCoral(endEffector),
                                                              new WaitCommand(0.5))),
            new ZeroElevator(elevator),
            new ParallelCommandGroup(
                new DrivePath(drivetrain, path2, localizer),
                new LoadCoral(endEffector)
            ),
            new WaitCommand(1),
            new DrivePath(drivetrain, path3, localizer),
            new CoralElevatorToHeight(elevator, branchLevel, true),
            new ParallelRaceGroup( new CoralElevatorToHeight(elevator, branchLevel, false),
            new SequentialCommandGroup(new ScoreCoral(endEffector),
                                       new WaitCommand(0.5)))
        );
    }
}
