// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.CoralEndeffector;
import frc.robot.commands.AlgaeGrab;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandStates;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.Lidar;
import frc.robot.subsystems.Localizer;

/** Add your docs here. */
public class AutoCenterStart 
{
    public static Command create(int level, boolean isRed, Drivetrain drivetrain, Localizer localizer, FieldMap map, Climber climber, CoralEndeffector endEffector, CoralElevator elevator, Lidar lidar, AprilTagFinder finder, AlgaePivot algaePivot, CommandStates state, boolean xversion)
    {
        switch (level)
        {
            case 0: 
               return Leave.create(isRed, drivetrain, localizer, climber);
            case 1: 
               return CenterScoreCoral.create(isRed, drivetrain, map, localizer, endEffector, elevator, lidar, finder, state, 1);
            case 2:
               if (xversion) return CenterScoreCoralX.create(isRed, drivetrain, map, localizer, endEffector, elevator, lidar, 2);
               else return CenterScoreCoral.create(isRed, drivetrain, map, localizer, endEffector, elevator, lidar, finder, state, 2);
            case 3:
               if (xversion) return CenterScoreCoralX.create(isRed, drivetrain, map, localizer, endEffector, elevator, lidar, 3);
               else return CenterScoreCoral.create(isRed, drivetrain, map, localizer, endEffector, elevator, lidar, finder, state, 3);
            case 4:
               if (xversion) return CenterScoreCoralX.create(isRed, drivetrain, map, localizer, endEffector, elevator, lidar, 4);
               else return CenterScoreCoral.create(isRed, drivetrain, map, localizer, endEffector, elevator, lidar, finder, state, 4);
            case 5:
               return CenterScore2Coral.create(isRed, drivetrain, map, localizer, endEffector, elevator, lidar, 4);
            case 6:
               return GrabAlgae.create(isRed, drivetrain, map, localizer, endEffector, elevator, state, algaePivot, finder, lidar, 4);
            case 7:
               return BargeScore.create(isRed, drivetrain, map, localizer, endEffector, elevator, state, algaePivot, finder, lidar, 4);
            default:
               return new WaitCommand(0);
        }
    }
}