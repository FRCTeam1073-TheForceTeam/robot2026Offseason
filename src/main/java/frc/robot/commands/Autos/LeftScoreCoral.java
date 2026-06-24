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
import frc.robot.commands.DrivePath;
import frc.robot.commands.LoadCoral;
import frc.robot.commands.Path;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.Path.Point;
import frc.robot.commands.Path.Segment;
import frc.robot.commands.ZeroElevator;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.CoralEndeffector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.Lidar;
import frc.robot.subsystems.Localizer;

public class LeftScoreCoral 
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

        Pose2d tag11Pose = map.getTagRelativePose(11, slot, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d tag11ApproachPose = map.getTagRelativePose(11, slot, new Transform2d(AutoConstants.scoreApproachOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d redIntermediatePose = map.getTagRelativePose(11, 0, new Transform2d(AutoConstants.intermediateOffsetX, AutoConstants.intermediateOffsetY, new Rotation2d(Math.PI)));
        Pose2d tag1Pose = map.getTagRelativePose(1, 2, new Transform2d(AutoConstants.loadOffsetX, 0, new Rotation2d()));

        Pose2d tag20Pose = map.getTagRelativePose(20, slot, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d tag20ApproachPose = map.getTagRelativePose(20, slot, new Transform2d(AutoConstants.scoreApproachOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d blueIntermediatePose = map.getTagRelativePose(20, 0, new Transform2d(AutoConstants.intermediateOffsetX, AutoConstants.intermediateOffsetY, new Rotation2d(Math.PI)));
        Pose2d tag13Pose = map.getTagRelativePose(13, 2, new Transform2d(AutoConstants.loadOffsetX, 0, new Rotation2d()));

        Point start = new Point(localizer.getPose().getX(), localizer.getPose().getY());

        Point tag11 = new Point(tag11Pose.getX(), tag11Pose.getY());
        tag11.blend_radius = AutoConstants.blendRadius;
        Point tag11Approach = new Point(tag11ApproachPose.getX(), tag11ApproachPose.getY() - 0.2);
        // tag11Approach.blend_radius = AutoConstants.blendRadius;
        Point redI1 = new Point(redIntermediatePose.getX(), redIntermediatePose.getY());
        // redI1.blend_radius = AutoConstants.blendRadius;
        Point tag1 = new Point(tag1Pose.getX(), tag1Pose.getY());
        tag1.blend_radius = AutoConstants.blendRadius;

        Point tag20 = new Point(tag20Pose.getX(), tag20Pose.getY());
        tag20.blend_radius = AutoConstants.blendRadius;
        Point tag20Approach = new Point(tag20ApproachPose.getX(), tag20ApproachPose.getY() - 0.2);
        // tag20Approach.blend_radius = AutoConstants.blendRadius;
        Point blueI1 = new Point(blueIntermediatePose.getX(), blueIntermediatePose.getY());
        // blueI1.blend_radius = AutoConstants.blendRadius;
        Point tag13 = new Point(tag13Pose.getX(), tag13Pose.getY());
        tag13.blend_radius = AutoConstants.blendRadius;

        ArrayList<Segment> segments1 = new ArrayList<Segment>();
        ArrayList<Segment> segments2 = new ArrayList<Segment>();

        Path path1;
        Path path2;

        if (isRed)
        {
            segments1.add(new Segment(start, tag11Approach, tag11ApproachPose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));
            segments1.add(new Segment(tag11Approach, tag11, tag11Pose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

            segments2.add(new Segment(tag11, redI1, redIntermediatePose.getRotation().getRadians(), AutoConstants.stowingElevatorVelocity));
            segments2.add(new Segment(redI1, tag1, tag1Pose.getRotation().getRadians(), AutoConstants.stowedDrivingVelocity));

            path1 = new Path(segments1, tag11Pose.getRotation().getRadians());
            path2 = new Path(segments2, tag1Pose.getRotation().getRadians());
        }
        else
        {
            segments1.add(new Segment(start, tag20Approach, tag20ApproachPose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));
            segments1.add(new Segment(tag20Approach, tag20, tag20Pose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

            segments2.add(new Segment(tag20, blueI1, blueIntermediatePose.getRotation().getRadians(), AutoConstants.stowingElevatorVelocity));
            segments2.add(new Segment(blueI1, tag13, tag13Pose.getRotation().getRadians(), AutoConstants.stowedDrivingVelocity));

            path1 = new Path(segments1, tag20Pose.getRotation().getRadians());
            path2 = new Path(segments2, tag13Pose.getRotation().getRadians());
        }
        

        return new SequentialCommandGroup(
            // TODO: Load and drive should be parallel. Every second counts.
            new LoadCoral(endEffector),
            new DrivePath(drivetrain, path1, localizer),
            // new LidarAlign(lidar, drivetrain),
            new CoralElevatorToHeight(elevator, branchLevel, true),
            new ParallelRaceGroup( new CoralElevatorToHeight(elevator, branchLevel, false),
                                   new SequentialCommandGroup(new ScoreCoral(endEffector),
                                                              new WaitCommand(0.5))),
            new ParallelCommandGroup(
                new ZeroElevator(elevator),
                new DrivePath(drivetrain, path2, localizer)
            ),
            new LoadCoral(endEffector)
        );
    }     
}
