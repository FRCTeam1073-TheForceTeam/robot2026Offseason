// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralEndeffector;

/** Add your docs here. */
public class AlgaeHold {
    public static Command create(AlgaePivot pivot, CoralEndeffector endeffector){
        return new ConditionalCommand(
            new InstantCommand(),
            new ParallelCommandGroup(
                new AlgaeGrab(endeffector, false),
                new HoldPivotPosition(pivot)
            ),
                CoralEndeffector::getHasCoral
        );
    }
}
