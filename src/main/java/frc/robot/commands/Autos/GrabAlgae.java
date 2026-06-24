// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.robot.commands.AlgaeAutoGrab;
import frc.robot.commands.AlgaeGrab;
import frc.robot.commands.AlgaeOpenAuto;
import frc.robot.commands.AlgaePivotGrab;
import frc.robot.commands.AlgaePivotToPosition;
import frc.robot.commands.AlignToTagRelative;
import frc.robot.commands.CoralElevatorToHeight;
import frc.robot.commands.DetectElevatorHeight;
import frc.robot.commands.DrivePath;
import frc.robot.commands.HoldPivotPosition;
import frc.robot.commands.LidarAlign;
import frc.robot.commands.LoadAlgaeAuto;
import frc.robot.commands.LoadCoral;
import frc.robot.commands.Path;
import frc.robot.commands.ScoreAlgaeAuto;
import frc.robot.commands.Path.Point;
import frc.robot.commands.Path.Segment;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.ZeroAlgaePivot;
import frc.robot.commands.ZeroElevator;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.CommandStates;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.CoralEndeffector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.Lidar;
import frc.robot.subsystems.Localizer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GrabAlgae extends Command {
  /** Creates a new CenterCoralAndBarge. */
  public static Command create(boolean isRed, Drivetrain drivetrain, FieldMap map, Localizer localizer, CoralEndeffector endEffector, CoralElevator elevator, CommandStates state, AlgaePivot algaePivot, AprilTagFinder finder, Lidar lidar, int branchLevel) {
    int height;
    int slot;
    if (branchLevel == 1)
    {
      slot = 0;
    }
    else
    {
      slot = -1;
    }

    // tag 14 is the blue barge half from the blue side
    // tag 15 is the red barge half from the blue side, thus we do not use it
    // conversely for tag 4
    // and tag 5 is the red half from the red side

    Pose2d tag10Pose = map.getTagRelativePose(10, slot, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
    Pose2d tag10ApproachPose = map.getTagRelativePose(10, slot, new Transform2d(AutoConstants.scoreApproachOffsetX, -AutoConstants.scoreApproachOffsetY, new Rotation2d(Math.PI)));
    Pose2d tag10AlgaePose = map.getTagRelativePose(10, 0, new Transform2d(AutoConstants.algaeOffsetX, 0, new Rotation2d(Math.PI)));

    Pose2d tag21Pose = map.getTagRelativePose(21, slot, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI))); 
    Pose2d tag21ApproachPose = map.getTagRelativePose(21, slot, new Transform2d(AutoConstants.scoreApproachOffsetX, -AutoConstants.scoreApproachOffsetY, new Rotation2d(Math.PI)));
    Pose2d tag21AlgaePose = map.getTagRelativePose(21, 0, new Transform2d(AutoConstants.algaeOffsetX, 0, new Rotation2d(Math.PI)));

    Point start = new Point(localizer.getPose().getX(), localizer.getPose().getY());

    

    Pose2d tag14Pose = map.getTagRelativePose(14, 0, new Transform2d(AutoConstants.algaeScoreOffsetX, 0, new Rotation2d(Math.PI)));
    // Pose2d tag15Pose = map.getTagRelativePose(15, 0, new Transform2d(AutoConstants.algaeScoreOffsetX, 0, new Rotation2d(Math.PI))); //TODO: implement a way to pick which side go to
    // Pose2d tag4Pose = map.getTagRelativePose(4, 0, new Transform2d(AutoConstants.algaeScoreOffsetX, 0, new Rotation2d(Math.PI)));
    Pose2d tag5Pose = map.getTagRelativePose(5, 0, new Transform2d(AutoConstants.algaeScoreOffsetX, 0, new Rotation2d(Math.PI)));

    Pose2d tag14EndPose = map.getTagRelativePose(14, 0, new Transform2d(AutoConstants.algaeEndOffsetX, 0, new Rotation2d(Math.PI)));
    Pose2d tag5EndPose = map.getTagRelativePose(5, 0, new Transform2d(AutoConstants.algaeEndOffsetX, 0, new Rotation2d(Math.PI)));

    Point tag10 = new Point(tag10Pose.getX(), tag10Pose.getY());
    tag10.blend_radius = AutoConstants.blendRadius;
    Point tag10Approach = new Point(tag10ApproachPose.getX(), tag10ApproachPose.getY());
    Point tag10Algae = new Point(tag10AlgaePose.getX(), tag10AlgaePose.getY());

    Point tag21 = new Point(tag21Pose.getX(), tag21Pose.getY());
    tag21.blend_radius = AutoConstants.blendRadius;
    Point tag21Approach = new Point(tag21ApproachPose.getX(), tag21ApproachPose.getY());
    Point tag21Algae = new Point(tag21AlgaePose.getX(), tag21AlgaePose.getY());

    Point tag14 = new Point(tag14Pose.getX(), tag14Pose.getY());
    tag14.blend_radius = AutoConstants.blendRadius;
    // Point tag15 = new Point(tag15Pose.getX(), tag15Pose.getY());
    // tag15.blend_radius = AutoConstants.blendRadius;
    // Point tag4 = new Point(tag4Pose.getX(), tag4Pose.getY());
    // tag4.blend_radius = AutoConstants.blendRadius;
    Point tag5 = new Point(tag5Pose.getX(), tag5Pose.getY());
    tag5.blend_radius = AutoConstants.blendRadius;

    Point tag5End = new Point(tag5EndPose.getX(), tag5EndPose.getY());
    Point tag14End = new Point(tag14EndPose.getX(), tag14EndPose.getY());

    ArrayList<Segment> segments = new ArrayList<Segment>();
    ArrayList<Segment> segments2 = new ArrayList<Segment>();
    ArrayList<Segment> segments3 = new ArrayList<Segment>();

    Path path;
    Path path2;
    Path path3;

    int tagID;

    if(isRed) 
    {
      segments.add(new Segment(start, tag10Approach, tag10ApproachPose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));
      segments2.add(new Segment(tag10, tag10Algae, tag10AlgaePose.getRotation().getRadians(), AutoConstants.reefApproachVelocity));
      segments3.add(new Segment(tag10, tag10Algae, tag10AlgaePose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

      path = new Path(segments, tag10ApproachPose.getRotation().getRadians());
      path2 = new Path(segments2, tag10AlgaePose.getRotation().getRadians());
      path3 = new Path(segments3, tag10AlgaePose.getRotation().getRadians());
      tagID = 10;

    }
    else 
    {
      segments.add(new Segment(start, tag21Approach, tag21ApproachPose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));
      segments2.add(new Segment(tag21, tag21Algae, tag21AlgaePose.getRotation().getRadians(), AutoConstants.reefApproachVelocity));
      segments3.add(new Segment(tag21, tag21Algae, tag21AlgaePose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

      path = new Path(segments, tag21ApproachPose.getRotation().getRadians());
      path2 = new Path(segments2, tag21AlgaePose.getRotation().getRadians());
      path3 = new Path(segments3, tag21AlgaePose.getRotation().getRadians());
      tagID = 21;
    }

    if(FieldMap.algaeHeight.get((isRed) ? 10 : 21) == 0) 
    {
      height = 5;
    }
    else 
    {
      height = 6;
    }

    return new SequentialCommandGroup(
      new ParallelRaceGroup(
        new SequentialCommandGroup( // scores coral 
          new ParallelCommandGroup( 
            new LoadCoral(endEffector),
            new DrivePath(drivetrain, path, localizer) 
          ),
          new AlignToTagRelative(drivetrain, finder, state, tagID, slot), 
          new CoralElevatorToHeight(elevator, branchLevel, true),
          new ParallelRaceGroup( 
            new CoralElevatorToHeight(elevator, branchLevel, false),
            new SequentialCommandGroup(
              new ScoreCoral(endEffector),
              new WaitCommand(AutoConstants.elevatorDelay) 
            )
          ),
          new ParallelCommandGroup(
            new ZeroElevator(elevator),
            new DrivePath(drivetrain, path2, localizer),
            new AlgaePivotToPosition(algaePivot, 10, true)
            //new AlgaeGrab(endEffector, true)
          ),
          // AlgaeAutoGrab.create(algaePivot, endEffector),
          new SequentialCommandGroup(
            new ParallelRaceGroup(
              new SequentialCommandGroup(
                new AlignToTagRelative(drivetrain, finder, state, tagID, 0),
                new ParallelRaceGroup(
                  new WaitCommand(1.5),
                  new LidarAlign(lidar, drivetrain, state)
                )
              ),
              new AlgaeGrab(endEffector, false)
            ),
            new ParallelRaceGroup(
              new WaitCommand(2),
              new AlgaeGrab(endEffector, false),
              new AlgaePivotToPosition(algaePivot, 6.5, true)
            ),
            new ParallelRaceGroup(
              new HoldPivotPosition(algaePivot),
              new DrivePath(drivetrain, path3, localizer)
            )
          )
        )
          
          // new ParallelCommandGroup(       old ending 4/17/25 worlds
          //  // AlgaeAutoGrab.create(algaePivot, endEffector),
          //   new SequentialCommandGroup(
          //     new WaitCommand(1),
          //     new AlignToTagRelative(drivetrain, finder, state, tagID, 0),
          //     new ParallelRaceGroup(
          //       new WaitCommand(2),
          //       new LidarAlign(lidar, drivetrain, state)
          //     ),
          //     new ParallelCommandGroup(
          //         new AlgaeGrab(endEffector, true),
          //         new AlgaePivotGrab(algaePivot)
          //     ),
          //     new ParallelRaceGroup(
          //       new HoldPivotPosition(algaePivot),
          //       new DrivePath(drivetrain, path3, localizer)
          //     )
          //   )
          // )
        )
      );
  }
}
