
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralEndeffector;
import frc.robot.subsystems.OI;

/** Add your docs here. */
public class AlgaeOpenAuto {
    public static Command create(AlgaePivot pivot, CoralEndeffector endeffector){
        return new SequentialCommandGroup(
            new ZeroAlgaePivot(pivot),
            new AlgaePivotToPosition(pivot, 11, true),
            new AlgaeGrab(endeffector, false),

            new ParallelCommandGroup(
                new AlgaeGrab(endeffector, false),
                new AlgaePivotToPosition(pivot, 9, true)
            )
        );
    }
}
