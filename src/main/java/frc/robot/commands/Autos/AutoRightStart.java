// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.CoralEndeffector;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandStates;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.Lidar;
import frc.robot.subsystems.Localizer;

/** Add your docs here. */
public class AutoRightStart 
{
    public static Command create(int level, boolean isRed, Drivetrain drivetrain, Localizer localizer, FieldMap map, Climber climber, CoralEndeffector endEffector, CoralElevator elevator, Lidar lidar, AprilTagFinder finder, CommandStates state) 
    {
        switch(level) 
        {
            case 0: 
               return Leave.create(isRed, drivetrain, localizer, climber);
            case 1: 
                return RightScoreCoral.create(isRed, drivetrain, map, localizer, endEffector, elevator, lidar, 1);
            case 2:
                return RightScoreCoral.create(isRed, drivetrain, map, localizer, endEffector, elevator, lidar, 2);
            case 3:
                return RightScoreCoral.create(isRed, drivetrain, map, localizer, endEffector, elevator, lidar, 3);
            case 4:
                return RightScoreCoral.create(isRed, drivetrain, map, localizer, endEffector, elevator, lidar, 4);
            case 5:
                return RightScore2Coral.create(isRed, drivetrain, map, localizer, endEffector, elevator, finder, lidar, state, 4);
            case 11:
                return RightScore3Coral.create(isRed, drivetrain, map, localizer, endEffector, elevator, lidar, finder, state, 4);
            default:
                return new WaitCommand(0);
        }
    }
}
