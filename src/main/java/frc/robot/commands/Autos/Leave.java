package frc.robot.commands.Autos;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.DrivePath;
import frc.robot.commands.Path;
import frc.robot.commands.ZeroClimber;
import frc.robot.commands.Path.Point;
import frc.robot.commands.Path.Segment;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Localizer;

public class Leave 
{
    public static Command create(boolean isRed, Drivetrain drivetrain, Localizer localizer, Climber climber)
    {
        int allianceSign;
        double allianceOrientation;

        if (isRed)
        {
            allianceSign = 1;
            allianceOrientation = 0;
        }
        else 
        {
            allianceSign = -1;
            allianceOrientation = Math.PI;
        }

        Point start = new Point(localizer.getPose().getX(), localizer.getPose().getY());
        Point end = new Point(localizer.getPose().getX() + 1.0 * allianceSign, localizer.getPose().getY());
        end.blend_radius = AutoConstants.blendRadius;

        ArrayList<Segment> segments = new ArrayList<Segment>();

        segments.add(new Segment(start, end, allianceOrientation, 1));

        Path path = new Path(segments, allianceOrientation);

        return new ParallelCommandGroup(
            new DrivePath(drivetrain, path, localizer), new ZeroClimber(climber)
        );
    }    
}
