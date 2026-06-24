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

public class LeftScore2Coral 
{
    public static Command create(boolean isRed, Drivetrain drivetrain, FieldMap map, Localizer localizer, CoralEndeffector endEffector, CoralElevator elevator, Lidar lidar, AprilTagFinder finder, CommandStates state, int branchLevel)  
    {
        Pose2d tag6LeftPose = map.getTagRelativePose(6, -1, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d tag11RightPose = map.getTagRelativePose(11, 1, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d tag11RightApproachPose = map.getTagRelativePose(11, 1, new Transform2d(AutoConstants.scoreApproachOffsetX, -AutoConstants.scoreApproachOffsetY, new Rotation2d(Math.PI)));
        Pose2d tag6LeftApproachPose = map.getTagRelativePose(6, -1, new Transform2d(AutoConstants.scoreApproachOffsetX, -AutoConstants.scoreApproachOffsetY, new Rotation2d(Math.PI)));
        Pose2d redIntermediatePose = map.getTagRelativePose(11, 0, new Transform2d(AutoConstants.intermediateOffsetX, AutoConstants.intermediateOffsetY, new Rotation2d(Math.PI)));
        Pose2d tag1Pose = map.getTagRelativePose(1, 2, new Transform2d(AutoConstants.loadOffsetX, 0, new Rotation2d()));

        Pose2d tag19LeftPose = map.getTagRelativePose(19, -1, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d tag20RightPose = map.getTagRelativePose(20, 1, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d tag20RightApproachPose = map.getTagRelativePose(20, 1, new Transform2d(AutoConstants.scoreApproachOffsetX, -AutoConstants.scoreApproachOffsetY, new Rotation2d(Math.PI)));
        Pose2d tag19LeftApproachPose = map.getTagRelativePose(19, -1, new Transform2d(AutoConstants.scoreApproachOffsetX, -AutoConstants.scoreApproachOffsetY, new Rotation2d(Math.PI)));
        Pose2d blueIntermediatePose = map.getTagRelativePose(20, 0, new Transform2d(AutoConstants.intermediateOffsetX, AutoConstants.intermediateOffsetY, new Rotation2d(Math.PI)));
        Pose2d tag13Pose = map.getTagRelativePose(13, 2, new Transform2d(AutoConstants.loadOffsetX, 0, new Rotation2d()));

        Point start = new Point(localizer.getPose().getX(), localizer.getPose().getY());

        Point tag6L = new Point(tag6LeftPose.getX(), tag6LeftPose.getY());
        tag6L.blend_radius = AutoConstants.blendRadius;
        Point tag6LApproach = new Point(tag6LeftApproachPose.getX(), tag6LeftApproachPose.getY());
        Point tag11RApproach = new Point(tag11RightApproachPose.getX(), tag11RightApproachPose.getY());
        Point redI1 = new Point(redIntermediatePose.getX(), redIntermediatePose.getY());
        // redI1.blend_radius = AutoConstants.blendRadius;
        Point tag1 = new Point(tag1Pose.getX(), tag1Pose.getY());
        tag1.blend_radius = AutoConstants.blendRadius;
        Point tag11R = new Point(tag11RightPose.getX(), tag11RightPose.getY());
        tag11R.blend_radius = AutoConstants.blendRadius;


        Point tag19L = new Point(tag19LeftPose.getX(), tag19LeftPose.getY());
        tag19L.blend_radius = AutoConstants.blendRadius;
        Point tag19LApproach = new Point(tag19LeftApproachPose.getX(), tag19LeftApproachPose.getY());
        Point tag20RApproach = new Point(tag20RightApproachPose.getX(), tag20RightApproachPose.getY());
        Point blueI1 = new Point(blueIntermediatePose.getX(), blueIntermediatePose.getY());
        // blueI1.blend_radius = AutoConstants.blendRadius;
        Point tag13 = new Point(tag13Pose.getX(), tag13Pose.getY());
        tag13.blend_radius = AutoConstants.blendRadius;
        Point tag20R = new Point(tag20RightPose.getX(), tag20RightPose.getY());
        tag20R.blend_radius = AutoConstants.blendRadius;

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
            segments1.add(new Segment(start, tag11RApproach, tag11RightApproachPose.getRotation().getRadians(), AutoConstants.reefApproachVelocity));
            // segments1.add(new Segment(tag11RApproach, tag11R, tag11RightPose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

            segments2.add(new Segment(tag11R, redI1, redIntermediatePose.getRotation().getRadians(), AutoConstants.stowingElevatorVelocity));
            segments2.add(new Segment(redI1, tag1, tag1Pose.getRotation().getRadians(), AutoConstants.stowedDrivingVelocity));

            // segments3.add(new Segment(tag1, redI1, redIntermediatePose.getRotation().getRadians(), AutoConstants.stowedDrivingVelocity));
            segments3.add(new Segment(tag1, tag6LApproach, tag6LeftApproachPose.getRotation().getRadians(), AutoConstants.stowedDrivingVelocity));
            // segments3.add(new Segment(tag11RApproach, tag11R, tag11RightPose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

            segments4.add(new Segment(tag6L, tag1, tag1Pose.getRotation().getRadians(), AutoConstants.stowingElevatorVelocity));

            path1 = new Path(segments1, tag11RightPose.getRotation().getRadians());
            path2 = new Path(segments2, tag1Pose.getRotation().getRadians());
            path3 = new Path(segments3, tag6LeftApproachPose.getRotation().getRadians());
            path4 = new Path(segments4, tag1Pose.getRotation().getRadians());

            localTagID = 11;
            localTagID2 = 6;
        }
        else
        {
            segments1.add(new Segment(start, tag20RApproach, tag20RightApproachPose.getRotation().getRadians(), AutoConstants.reefApproachVelocity));
            // segments1.add(new Segment(tag20RApproach, tag20R, tag20RightPose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

            segments2.add(new Segment(tag20R, blueI1, blueIntermediatePose.getRotation().getRadians(), AutoConstants.stowingElevatorVelocity));
            segments2.add(new Segment(blueI1, tag13, tag13Pose.getRotation().getRadians(), AutoConstants.stowedDrivingVelocity));

            // segments3.add(new Segment(tag13, blueI1, blueIntermediatePose.getRotation().getRadians(), AutoConstants.stowedDrivingVelocity));
            segments3.add(new Segment(tag13, tag19LApproach, tag19LeftApproachPose.getRotation().getRadians(), AutoConstants.stowedDrivingVelocity));
            // segments3.add(new Segment(tag20RApproach, tag20R, tag20RightPose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

            segments4.add(new Segment(tag19L, tag13, tag13Pose.getRotation().getRadians(), AutoConstants.stowingElevatorVelocity));

            path1 = new Path(segments1, tag20RightPose.getRotation().getRadians());
            path2 = new Path(segments2, tag13Pose.getRotation().getRadians());
            path3 = new Path(segments3, tag19LeftApproachPose.getRotation().getRadians());
            path4 = new Path(segments4, tag13Pose.getRotation().getRadians());

            localTagID = 20;
            localTagID2 = 19;
        }
        

        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new LoadCoral(endEffector),
                new DrivePath(drivetrain, path1, localizer),
                new CoralElevatorToHeight(elevator, 2, true)
            ),
            new ParallelDeadlineGroup(
                new AlignToTagRelative(drivetrain, finder, state, localTagID, 1),
                new CoralElevatorToHeight(elevator, 4, true)
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
            // TODO: Load and drive in parallel. Every second counts.
            new LoadCoral(endEffector),
            new DrivePath(drivetrain, path3, localizer),
            new AlignToTagRelative(drivetrain, finder, state, localTagID2, -1),
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
