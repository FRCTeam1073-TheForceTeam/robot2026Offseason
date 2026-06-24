package frc.robot.commands.Autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DrivePath;
import frc.robot.commands.Path;
import frc.robot.commands.Path.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.Localizer;

public class TestAuto 
{
    public static Command create(Drivetrain drivetrain, Localizer localizer, FieldMap map)
    {
        Point start = new Point(7.5, 6);
        Point tag20 = new Point(map.getTagRelativePose(20, 0, new Transform2d(1.5, 0, new Rotation2d())).getX(), map.getTagRelativePose(20, 0, new Transform2d(1.5, 0, new Rotation2d())).getY());
        Point tag19 = new Point(map.getTagRelativePose(19, 0, new Transform2d(1.5, 0, new Rotation2d())).getX(), map.getTagRelativePose(19, 0, new Transform2d(1.5, 0, new Rotation2d())).getY());
        Point tag18 = new Point(map.getTagRelativePose(18, 0, new Transform2d(1.5, 0, new Rotation2d())).getX(), map.getTagRelativePose(18, 0, new Transform2d(1.5, 0, new Rotation2d())).getY());
        Point tag17 = new Point(map.getTagRelativePose(17, 0, new Transform2d(1.5, 0, new Rotation2d())).getX(), map.getTagRelativePose(17, 0, new Transform2d(1.5, 0, new Rotation2d())).getY());
        Point tag22 = new Point(map.getTagRelativePose(22, 0, new Transform2d(1.5, 0, new Rotation2d())).getX(), map.getTagRelativePose(22, 0, new Transform2d(1.5, 0, new Rotation2d())).getY());
        Point tag21 = new Point(map.getTagRelativePose(21, 0, new Transform2d(1.5, 0, new Rotation2d())).getX(), map.getTagRelativePose(21, 0, new Transform2d(1.5, 0, new Rotation2d())).getY());

        ArrayList<Segment> segments = new ArrayList<Segment>();
        segments.add(new Segment(start, tag20, Math.PI, 2));
        segments.add(new Segment(tag20, tag19, Math.PI, 2));
        segments.add(new Segment(tag19, tag18, Math.PI, 2));
        segments.add(new Segment(tag18, tag17, Math.PI, 2));
        segments.add(new Segment(tag17, tag22, Math.PI, 2));
        segments.add(new Segment(tag22, tag21, Math.PI, 2));
        segments.add(new Segment(tag21, start, Math.PI, 2));

        Path path = new Path(segments, Math.PI);
        return new SequentialCommandGroup(new DrivePath(drivetrain, path, localizer));
    }
}
