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

public class CenterScoreCoralX 
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
        Pose2d tag21Pose = map.getTagRelativePose(21, slot, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
        Point start = new Point(localizer.getPose().getX(), localizer.getPose().getY());
        Point tag10 = new Point(tag10Pose.getX(), tag10Pose.getY());
        tag10.blend_radius = AutoConstants.blendRadius;
        Point tag21 = new Point(tag21Pose.getX(), tag21Pose.getY());
        tag21.blend_radius = AutoConstants.blendRadius;


        ArrayList<Segment> segments = new ArrayList<Segment>();
        Path path;
        if (isRed)
        {
            segments.add(new Segment(start, tag10, tag10Pose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

            path = new Path(segments, tag10Pose.getRotation().getRadians());
        }
        else
        {
            segments.add(new Segment(start, tag21, tag21Pose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

            path = new Path(segments, tag21Pose.getRotation().getRadians());
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
                                   new SequentialCommandGroup(
                                    new CreepToReef(drivetrain, endEffector, -1).withTimeout(3.0),
                                                            new ScoreCoral(endEffector),
                                                            new WaitCommand(0.5))),
            new WaitCommand(0.5),
            new ZeroElevator(elevator)
        );
    }
}
