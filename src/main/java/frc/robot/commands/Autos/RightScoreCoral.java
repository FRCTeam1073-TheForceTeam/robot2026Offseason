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

public class RightScoreCoral 
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

        Pose2d tag9Pose = map.getTagRelativePose(9, slot, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d tag9ApproachPose = map.getTagRelativePose(9, slot, new Transform2d(AutoConstants.scoreApproachOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d redIntermediatePose = map.getTagRelativePose(9, 0, new Transform2d(AutoConstants.intermediateOffsetX, -AutoConstants.intermediateOffsetY, new Rotation2d(Math.PI)));
        Pose2d tag2Pose = map.getTagRelativePose(2, 2, new Transform2d(AutoConstants.loadOffsetX, 0, new Rotation2d()));

        Pose2d tag22Pose = map.getTagRelativePose(22, slot, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d tag22ApproachPose = map.getTagRelativePose(22, slot, new Transform2d(AutoConstants.scoreApproachOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d blueIntermediatePose = map.getTagRelativePose(22, 0, new Transform2d(AutoConstants.intermediateOffsetX, -AutoConstants.intermediateOffsetY, new Rotation2d(Math.PI)));
        Pose2d tag12Pose = map.getTagRelativePose(12, 2, new Transform2d(AutoConstants.loadOffsetX, 0, new Rotation2d()));

        Point start = new Point(localizer.getPose().getX(), localizer.getPose().getY());

        Point tag9 = new Point(tag9Pose.getX(), tag9Pose.getY());
        tag9.blend_radius = AutoConstants.blendRadius;
        Point tag9Approach = new Point(tag9ApproachPose.getX(), tag9ApproachPose.getY() - 0.2);
        // tag9Approach.blend_radius = AutoConstants.blendRadius;
        Point redI1 = new Point(redIntermediatePose.getX(), redIntermediatePose.getY());
        // redI1.blend_radius = AutoConstants.blendRadius;
        Point tag2 = new Point(tag2Pose.getX(), tag2Pose.getY());
        tag2.blend_radius = AutoConstants.blendRadius;

        Point tag22 = new Point(tag22Pose.getX(), tag22Pose.getY());
        tag22.blend_radius = AutoConstants.blendRadius;
        Point tag22Approach = new Point(tag22ApproachPose.getX(), tag22ApproachPose.getY() - 0.2);
        // tag22Approach.blend_radius = AutoConstants.blendRadius;
        Point blueI1 = new Point(blueIntermediatePose.getX(), blueIntermediatePose.getY());
        // blueI1.blend_radius = AutoConstants.blendRadius;
        Point tag12 = new Point(tag12Pose.getX(), tag12Pose.getY());
        tag12.blend_radius = AutoConstants.blendRadius;

        ArrayList<Segment> segments1 = new ArrayList<Segment>();
        ArrayList<Segment> segments2 = new ArrayList<Segment>();

        Path path1;
        Path path2;

        if (isRed)
        {
            segments1.add(new Segment(start, tag9Approach, tag9ApproachPose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));
            segments1.add(new Segment(tag9Approach, tag9, tag9Pose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

            segments2.add(new Segment(tag9, redI1, redIntermediatePose.getRotation().getRadians(), AutoConstants.stowingElevatorVelocity));
            segments2.add(new Segment(redI1, tag2, tag2Pose.getRotation().getRadians(), AutoConstants.stowedDrivingVelocity));

            path1 = new Path(segments1, tag9Pose.getRotation().getRadians());
            path2 = new Path(segments2, tag2Pose.getRotation().getRadians());
        }
        else
        {
            segments1.add(new Segment(start, tag22Approach, tag22ApproachPose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));
            segments1.add(new Segment(tag22Approach, tag22, tag22Pose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

            segments2.add(new Segment(tag22, blueI1, blueIntermediatePose.getRotation().getRadians(), AutoConstants.stowingElevatorVelocity));
            segments2.add(new Segment(blueI1, tag12, tag12Pose.getRotation().getRadians(), AutoConstants.stowedDrivingVelocity));

            path1 = new Path(segments1, tag22Pose.getRotation().getRadians());
            path2 = new Path(segments2, tag12Pose.getRotation().getRadians());
        }
        

        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new LoadCoral(endEffector),
                new DrivePath(drivetrain, path1, localizer)
            ),
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
