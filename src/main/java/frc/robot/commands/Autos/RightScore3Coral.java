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
import frc.robot.commands.DetectCoral;
import frc.robot.commands.DriveBack;
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

public class RightScore3Coral 
{
    public static Command create(boolean isRed, Drivetrain drivetrain, FieldMap map, Localizer localizer, CoralEndeffector endEffector, CoralElevator elevator, Lidar lidar, AprilTagFinder finder, CommandStates state, int branchLevel)  
    {
        Pose2d tag8LeftPose = map.getTagRelativePose(8, 1, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d tag8RightPose = map.getTagRelativePose(8, -1, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d tag9RightPose = map.getTagRelativePose(9, -1, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d tag9RightApproachPose = map.getTagRelativePose(9, -1, new Transform2d(AutoConstants.scoreApproachOffsetX, -AutoConstants.scoreApproachOffsetY, new Rotation2d(Math.PI)));
        Pose2d tag8LeftApproachPose = map.getTagRelativePose(8, 1, new Transform2d(AutoConstants.scoreApproachOffsetX, -AutoConstants.scoreApproachOffsetY, new Rotation2d(Math.PI)));
        Pose2d tag8RightApproachPose = map.getTagRelativePose(8, -1, new Transform2d(AutoConstants.scoreApproachOffsetX, -AutoConstants.scoreApproachOffsetY, new Rotation2d(Math.PI)));
        Pose2d redIntermediatePose = map.getTagRelativePose(9, 0, new Transform2d(AutoConstants.intermediateOffsetX, -AutoConstants.intermediateOffsetY, new Rotation2d(Math.PI)));
        Pose2d tag2Pose = map.getTagRelativePose(2, 2, new Transform2d(AutoConstants.loadOffsetX, 0, new Rotation2d()));

        Pose2d tag17LeftPose = map.getTagRelativePose(17, 1, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d tag17RightPose = map.getTagRelativePose(17, -1, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d tag22RightPose = map.getTagRelativePose(22, -1, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d tag22RightApproachPose = map.getTagRelativePose(22, -1, new Transform2d(AutoConstants.scoreApproachOffsetX, -AutoConstants.scoreApproachOffsetY, new Rotation2d(Math.PI)));
        Pose2d tag17LeftApproachPose = map.getTagRelativePose(17, 1, new Transform2d(AutoConstants.scoreApproachOffsetX, -AutoConstants.scoreApproachOffsetY, new Rotation2d(Math.PI)));
        Pose2d tag17RightApproachPose = map.getTagRelativePose(17, -1, new Transform2d(AutoConstants.scoreApproachOffsetX, -AutoConstants.scoreApproachOffsetY, new Rotation2d(Math.PI)));
        Pose2d blueIntermediatePose = map.getTagRelativePose(22, 0, new Transform2d(AutoConstants.intermediateOffsetX, -AutoConstants.intermediateOffsetY, new Rotation2d(Math.PI)));
        Pose2d tag12Pose = map.getTagRelativePose(12, 2, new Transform2d(AutoConstants.loadOffsetX, 0, new Rotation2d()));

        Point start = new Point(localizer.getPose().getX(), localizer.getPose().getY());

        Point tag8L = new Point(tag8LeftPose.getX(), tag8LeftPose.getY());
        Point tag8R = new Point(tag8RightPose.getX(), tag8RightPose.getY());
        tag8L.blend_radius = AutoConstants.blendRadius;
        Point tag8LApproach = new Point(tag8LeftApproachPose.getX(), tag8LeftApproachPose.getY());
        Point tag8RApproach = new Point(tag8RightApproachPose.getX(), tag8RightApproachPose.getY());
        Point tag9RApproach = new Point(tag9RightApproachPose.getX(), tag9RightApproachPose.getY());
        Point redI1 = new Point(redIntermediatePose.getX(), redIntermediatePose.getY());
        // redI1.blend_radius = AutoConstants.blendRadius;
        Point tag2 = new Point(tag2Pose.getX(), tag2Pose.getY());
        tag2.blend_radius = AutoConstants.blendRadius;
        Point tag9R = new Point(tag9RightPose.getX(), tag9RightPose.getY());
        tag9R.blend_radius = AutoConstants.blendRadius;


        Point tag17L = new Point(tag17LeftPose.getX(), tag17LeftPose.getY());
        Point tag17R = new Point(tag17RightPose.getX(), tag17RightPose.getY());
        tag17L.blend_radius = AutoConstants.blendRadius;
        Point tag17LApproach = new Point(tag17LeftApproachPose.getX(), tag17LeftApproachPose.getY());
        Point tag17RApproach = new Point(tag17RightApproachPose.getX(), tag17RightApproachPose.getY());
        Point tag22RApproach = new Point(tag22RightApproachPose.getX(), tag22RightApproachPose.getY());
        Point blueI1 = new Point(blueIntermediatePose.getX(), blueIntermediatePose.getY());
        // blueI1.blend_radius = AutoConstants.blendRadius;
        Point tag12 = new Point(tag12Pose.getX(), tag12Pose.getY());
        tag12.blend_radius = AutoConstants.blendRadius;
        Point tag22R = new Point(tag22RightPose.getX(), tag22RightPose.getY());
        tag22R.blend_radius = AutoConstants.blendRadius;

        ArrayList<Segment> segments1 = new ArrayList<Segment>();
        ArrayList<Segment> segments2 = new ArrayList<Segment>();
        ArrayList<Segment> segments3 = new ArrayList<Segment>();
        ArrayList<Segment> segments4 = new ArrayList<Segment>();
        ArrayList<Segment> segments5 = new ArrayList<Segment>();

        Path path1;
        Path path2;
        Path path3;
        Path path4;
        Path path5;

        int localTagID;
        int localTagID2;

        if (isRed)
        {
            segments1.add(new Segment(start, tag9RApproach, tag9RightApproachPose.getRotation().getRadians(), AutoConstants.autoReefApproachVelocity));
            // segments1.add(new Segment(tag9RApproach, tag9R, tag9RightPose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

            segments2.add(new Segment(tag9R, redI1, redIntermediatePose.getRotation().getRadians(), AutoConstants.stowingElevatorVelocity));
            segments2.add(new Segment(redI1, tag2, tag2Pose.getRotation().getRadians(), AutoConstants.stowedDrivingVelocity));

            segments3.add(new Segment(tag2, tag8LApproach, tag8LeftApproachPose.getRotation().getRadians(), AutoConstants.stowedDrivingVelocity));
            
            segments4.add(new Segment(tag8L, tag2, tag2Pose.getRotation().getRadians(), AutoConstants.stowingElevatorVelocity));

            segments5.add(new Segment(tag2, tag8RApproach, tag8RightApproachPose.getRotation().getRadians(), AutoConstants.stowedDrivingVelocity));

            path1 = new Path(segments1, tag9RightPose.getRotation().getRadians());
            path2 = new Path(segments2, tag2Pose.getRotation().getRadians());
            path3 = new Path(segments3, tag8LeftApproachPose.getRotation().getRadians());
            path4 = new Path(segments4, tag2Pose.getRotation().getRadians());
            path5 = new Path(segments5, tag8RightApproachPose.getRotation().getRadians());

            localTagID = 9;
            localTagID2 = 8;
        }
        else
        {
            segments1.add(new Segment(start, tag22RApproach, tag22RightApproachPose.getRotation().getRadians(), AutoConstants.autoReefApproachVelocity));
            // segments1.add(new Segment(tag22RApproach, tag22R, tag22RightPose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

            segments2.add(new Segment(tag22R, blueI1, blueIntermediatePose.getRotation().getRadians(), AutoConstants.stowingElevatorVelocity));
            segments2.add(new Segment(blueI1, tag12, tag12Pose.getRotation().getRadians(), AutoConstants.stowedDrivingVelocity));

            segments3.add(new Segment(tag12, tag17LApproach, tag17LeftApproachPose.getRotation().getRadians(), AutoConstants.stowedDrivingVelocity));
            
            segments4.add(new Segment(tag17L, tag12, tag12Pose.getRotation().getRadians(), AutoConstants.stowingElevatorVelocity));

            segments5.add(new Segment(tag12, tag17RApproach, tag17RightApproachPose.getRotation().getRadians(), AutoConstants.stowedDrivingVelocity));
            
            path1 = new Path(segments1, tag22RightPose.getRotation().getRadians());
            path2 = new Path(segments2, tag12Pose.getRotation().getRadians());
            path3 = new Path(segments3, tag17LeftApproachPose.getRotation().getRadians());
            path4 = new Path(segments4, tag12Pose.getRotation().getRadians());
            path5 = new Path(segments5, tag17RightApproachPose.getRotation().getRadians());

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
            new ParallelRaceGroup(
                new AlignToTagRelative(drivetrain, finder, state, localTagID, -1),
                new CoralElevatorToHeight(elevator, 4, false)
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
            new ParallelRaceGroup(
                new LoadCoral(endEffector),
                new DetectCoral(endEffector)
            ),
            new ParallelCommandGroup(
                new DrivePath(drivetrain, path3, localizer),
                new SequentialCommandGroup(
                    new LoadCoral(endEffector),
                    new CoralElevatorToHeight(elevator, 2, true)
                )
            ),
            new ParallelRaceGroup(
                new AlignToTagRelative(drivetrain, finder, state, localTagID2, 1),
                new CoralElevatorToHeight(elevator, 4, false)
            ),
            new ParallelRaceGroup( new CoralElevatorToHeight(elevator, branchLevel, false),
                                   new SequentialCommandGroup(new ScoreCoral(endEffector),
                                                              new WaitCommand(AutoConstants.scoreDelay))),
            new ParallelCommandGroup(
                new ZeroElevator(elevator),
                new DrivePath(drivetrain, path4, localizer)
            ),
            new ParallelRaceGroup(
                new LoadCoral(endEffector),
                new DetectCoral(endEffector)
            ),
            new ParallelCommandGroup(
                new DrivePath(drivetrain, path5, localizer),
                new SequentialCommandGroup(
                    new LoadCoral(endEffector),
                    new CoralElevatorToHeight(elevator, 2, true)
                )
            ),
            new ParallelRaceGroup(
                new AlignToTagRelative(drivetrain, finder, state, localTagID2, -1),
                new CoralElevatorToHeight(elevator, 4, false)
            ),
            new ParallelRaceGroup( new CoralElevatorToHeight(elevator, branchLevel, false),
                                   new SequentialCommandGroup(new ScoreCoral(endEffector),
                                                              new WaitCommand(AutoConstants.scoreDelay)))
        );
    }     
}
