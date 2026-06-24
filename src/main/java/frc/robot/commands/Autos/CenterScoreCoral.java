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
import frc.robot.commands.AlignToTagRelative;
import frc.robot.commands.CoralElevatorToHeight;
import frc.robot.commands.DrivePath;
import frc.robot.commands.LoadCoral;
import frc.robot.commands.Path;
import frc.robot.commands.Path.Point;
import frc.robot.commands.Path.Segment;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.ZeroElevator;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.CommandStates;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.CoralEndeffector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.Lidar;
import frc.robot.subsystems.Localizer;

public class CenterScoreCoral 
{
    public static Command create(boolean isRed, Drivetrain drivetrain, FieldMap map, Localizer localizer, CoralEndeffector endEffector, CoralElevator elevator, Lidar lidar, AprilTagFinder finder, CommandStates state, int branchLevel)  
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
        Pose2d tag10ApproachPose = map.getTagRelativePose(10, slot, new Transform2d(AutoConstants.scoreApproachOffsetX, -AutoConstants.scoreApproachOffsetY, new Rotation2d(Math.PI)));

        Pose2d tag21Pose = map.getTagRelativePose(21, slot, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI))); 
        Pose2d tag21ApproachPose = map.getTagRelativePose(21, slot, new Transform2d(AutoConstants.scoreApproachOffsetX, -AutoConstants.scoreApproachOffsetY, new Rotation2d(Math.PI)));

        Point start = new Point(localizer.getPose().getX(), localizer.getPose().getY());

        Point tag10 = new Point(tag10Pose.getX(), tag10Pose.getY());
        tag10.blend_radius = AutoConstants.blendRadius;
        Point tag10Approach = new Point(tag10ApproachPose.getX(), tag10ApproachPose.getY());

        Point tag21 = new Point(tag21Pose.getX(), tag21Pose.getY());
        tag21.blend_radius = AutoConstants.blendRadius;
        Point tag21Approach = new Point(tag21ApproachPose.getX(), tag21ApproachPose.getY());


        ArrayList<Segment> segments = new ArrayList<Segment>();
        Path path;
        int tagID;

        if (isRed)
        {
            segments.add(new Segment(start, tag10Approach, tag10ApproachPose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

            path = new Path(segments, tag10ApproachPose.getRotation().getRadians());
            tagID = 10;
        }
        else
        {
            segments.add(new Segment(start, tag21Approach, tag21ApproachPose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

            path = new Path(segments, tag21ApproachPose.getRotation().getRadians());
            tagID = 21;
        }
        

        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new LoadCoral(endEffector),
                new DrivePath(drivetrain, path, localizer)
            ),
            new AlignToTagRelative(drivetrain, finder, state, tagID, slot),
            new CoralElevatorToHeight(elevator, branchLevel, true),
            new ParallelRaceGroup( new CoralElevatorToHeight(elevator, branchLevel, false),
                                   new SequentialCommandGroup(new ScoreCoral(endEffector),
                                                              new WaitCommand(AutoConstants.elevatorDelay))),
            new ZeroElevator(elevator)
        );
    }
}
