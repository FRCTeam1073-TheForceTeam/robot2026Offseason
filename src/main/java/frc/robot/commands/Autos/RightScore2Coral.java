package frc.robot.commands.Autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlignToTagRelative;
import frc.robot.commands.CoralElevatorToHeight;
import frc.robot.commands.DrivePath;
import frc.robot.commands.LoadCoral;
import frc.robot.commands.Path;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.Path.Point;
import frc.robot.commands.Path.Segment;
import frc.robot.commands.ZeroElevator;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.CommandStates;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.CoralEndeffector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.Lidar;
import frc.robot.subsystems.Localizer;

public class RightScore2Coral //UNTESTED!!!!!!!
{
    public static Command create(boolean isRed, Drivetrain drivetrain, FieldMap map, Localizer localizer, CoralEndeffector endEffector, CoralElevator elevator, AprilTagFinder finder, Lidar lidar, CommandStates state, int branchLevel)  
    {
        Pose2d tag8RightPose = map.getTagRelativePose(8, 1, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d tag9LeftPose = map.getTagRelativePose(9, -1, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d tag8RightApproachPose = map.getTagRelativePose(8, 1, new Transform2d(AutoConstants.scoreApproachOffsetX, -AutoConstants.scoreApproachOffsetY - 0.1, new Rotation2d(Math.PI)));
        Pose2d tag9LeftApproachPose = map.getTagRelativePose(9, -1, new Transform2d(AutoConstants.scoreApproachOffsetX, -AutoConstants.scoreApproachOffsetY, new Rotation2d(Math.PI)));
        Pose2d redIntermediatePose = map.getTagRelativePose(9, 0, new Transform2d(AutoConstants.intermediateOffsetX, -AutoConstants.intermediateOffsetY, new Rotation2d(Math.PI)));
        Pose2d tag2Pose = map.getTagRelativePose(2, 2, new Transform2d(AutoConstants.loadOffsetX, 0, new Rotation2d()));

        Pose2d tag17RightPose = map.getTagRelativePose(17, 1, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d tag22LeftPose = map.getTagRelativePose(22, -1, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d tag17RightApproachPose = map.getTagRelativePose(17, 1, new Transform2d(AutoConstants.scoreApproachOffsetX, -AutoConstants.scoreApproachOffsetY - 0.1, new Rotation2d(Math.PI)));
        Pose2d tag22LeftApproachPose = map.getTagRelativePose(22, -1, new Transform2d(AutoConstants.scoreApproachOffsetX, -AutoConstants.scoreApproachOffsetY, new Rotation2d(Math.PI)));
        Pose2d blueIntermediatePose = map.getTagRelativePose(22, 0, new Transform2d(AutoConstants.intermediateOffsetX, -AutoConstants.intermediateOffsetY, new Rotation2d(Math.PI)));
        Pose2d tag12Pose = map.getTagRelativePose(12, 2, new Transform2d(AutoConstants.loadOffsetX, 0, new Rotation2d()));

        Point start = new Point(localizer.getPose().getX(), localizer.getPose().getY());

        Point tag8R = new Point(tag8RightPose.getX(), tag8RightPose.getY());
        tag8R.blend_radius = AutoConstants.blendRadius;
        Point tag8RApproach = new Point(tag8RightApproachPose.getX(), tag8RightApproachPose.getY());
        Point tag9LApproach = new Point(tag9LeftApproachPose.getX(), tag9LeftApproachPose.getY());
        Point redI1 = new Point(redIntermediatePose.getX(), redIntermediatePose.getY());
        // redI1.blend_radius = AutoConstants.blendRadius;
        Point tag2 = new Point(tag2Pose.getX(), tag2Pose.getY());
        tag2.blend_radius = AutoConstants.blendRadius;
        Point tag9L = new Point(tag9LeftPose.getX(), tag9LeftPose.getY());
        tag9L.blend_radius = AutoConstants.blendRadius;

        Point tag17R = new Point(tag17RightPose.getX(), tag17RightPose.getY());
        tag17R.blend_radius = AutoConstants.blendRadius;
        Point tag17RApproach = new Point(tag17RightApproachPose.getX(), tag17RightApproachPose.getY());
        Point tag22LApproach = new Point(tag22LeftApproachPose.getX(), tag22LeftApproachPose.getY());
        Point blueI1 = new Point(blueIntermediatePose.getX(), blueIntermediatePose.getY());
        // blueI1.blend_radius = AutoConstants.blendRadius;
        Point tag12 = new Point(tag12Pose.getX(), tag12Pose.getY());
        tag12.blend_radius = AutoConstants.blendRadius;
        Point tag22L = new Point(tag22LeftPose.getX(), tag22LeftPose.getY());
        tag22L.blend_radius = AutoConstants.blendRadius;

        ArrayList<Segment> segments1 = new ArrayList<Segment>();
        ArrayList<Segment> segments2 = new ArrayList<Segment>();
        ArrayList<Segment> segments3 = new ArrayList<Segment>();
        ArrayList<Segment> segments4 = new ArrayList<Segment>();

        Path path1;
        Path path2;
        Path path3;
        Path path4;

        int localTagID;
        int localTagID2;

        if (isRed)
        {
            segments1.add(new Segment(start, tag9LApproach, tag9LeftApproachPose.getRotation().getRadians(), AutoConstants.reefApproachVelocity));
            // segments1.add(new Segment(tag9LApproach, tag9L, tag9LeftPose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

            segments2.add(new Segment(tag9L, redI1, tag2Pose.getRotation().getRadians(), AutoConstants.stowingElevatorVelocity));
            segments2.add(new Segment(redI1, tag2, tag2Pose.getRotation().getRadians(), AutoConstants.stowedDrivingVelocity));

            // segments3.add(new Segment(tag2, redI1, tag8RightApproachPose.getRotation().getRadians(), AutoConstants.stowedDrivingVelocity));
            segments3.add(new Segment(tag2, tag8RApproach, tag8RightApproachPose.getRotation().getRadians(), AutoConstants.stowedDrivingVelocity));
            // segments3.add(new Segment(tag9LApproach, tag9L, tag9LeftPose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

            segments4.add(new Segment(tag8R, tag2, tag2Pose.getRotation().getRadians(), AutoConstants.stowingElevatorVelocity));

            path1 = new Path(segments1, tag9LeftPose.getRotation().getRadians());
            path2 = new Path(segments2, tag2Pose.getRotation().getRadians());
            path3 = new Path(segments3, tag8RightApproachPose.getRotation().getRadians());
            path4 = new Path(segments4, tag2Pose.getRotation().getRadians());

            localTagID = 9;
            localTagID2 = 8;
        }
        else
        {
            segments1.add(new Segment(start, tag22LApproach, tag22LeftApproachPose.getRotation().getRadians(), AutoConstants.reefApproachVelocity));
            // segments1.add(new Segment(tag22LApproach, tag22L, tag22LeftPose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

            segments2.add(new Segment(tag22L, blueI1, tag12Pose.getRotation().getRadians(), AutoConstants.stowingElevatorVelocity));
            segments2.add(new Segment(blueI1, tag12, tag12Pose.getRotation().getRadians(), AutoConstants.stowedDrivingVelocity));

            // segments3.add(new Segment(tag12, blueI1, tag17RightApproachPose.getRotation().getRadians(), AutoConstants.stowedDrivingVelocity));
            segments3.add(new Segment(tag12, tag17RApproach, tag17RightApproachPose.getRotation().getRadians(), AutoConstants.stowedDrivingVelocity));
            // segments3.add(new Segment(tag22LApproach, tag22L, tag22LeftPose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

            segments4.add(new Segment(tag17R, tag12, tag12Pose.getRotation().getRadians(), AutoConstants.stowingElevatorVelocity));

            path1 = new Path(segments1, tag22LeftPose.getRotation().getRadians());
            path2 = new Path(segments2, tag12Pose.getRotation().getRadians());
            path3 = new Path(segments3, tag17RightApproachPose.getRotation().getRadians());
            path4 = new Path(segments4, tag12Pose.getRotation().getRadians());

            localTagID = 22;
            localTagID2 = 17;
        }
        

        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                new ParallelCommandGroup(
                    new LoadCoral(endEffector),
                    new DrivePath(drivetrain, path1, localizer)),
                new CoralElevatorToHeight(elevator, 2, true)
            ),
            new ParallelDeadlineGroup(
                new AlignToTagRelative(drivetrain, finder, state, localTagID, 1),
                new CoralElevatorToHeight(elevator, 3, true)
            ),
            new CoralElevatorToHeight(elevator, branchLevel, true),
            new ParallelRaceGroup( new CoralElevatorToHeight(elevator, branchLevel, false),
                                   new SequentialCommandGroup(new ScoreCoral(endEffector),
                                                              new WaitCommand(AutoConstants.scoreDelay))),
            new ParallelCommandGroup(
                new CoralElevatorToHeight(elevator, 5, true),
                new DrivePath(drivetrain, path2, localizer)
            ),
            // TODO: Consider using wait in stead of using load as wait.
            // TODO: Load and drive should be parallel. Every second counts.
            new LoadCoral(endEffector),
            new DrivePath(drivetrain, path3, localizer),
            new AlignToTagRelative(drivetrain, finder, state, localTagID2, 1),
            new CoralElevatorToHeight(elevator, branchLevel, true),
            new ParallelRaceGroup( new CoralElevatorToHeight(elevator, branchLevel, false),
                                   new SequentialCommandGroup(new ScoreCoral(endEffector),
                                                              new WaitCommand(AutoConstants.scoreDelay))),
            new ParallelCommandGroup(
                new ZeroElevator(elevator),
                new DrivePath(drivetrain, path4, localizer)
            )
            
        );
    }     
}
