// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.CoralEndeffector;

/** Add your docs here. */
public class AlgaeAutoGrab {
    public static Command create(AlgaePivot pivot, CoralEndeffector endeffector){
        /* 
         * 1. pivot out
         * 2. coral elevator to height
         * 3. roller until current spike
         * 4. pivot until current spike
         */
        return new SequentialCommandGroup(
            //TODO Fix TargetPosition
            //Zero, Find Algae, Load Algae, Eject, Zero
            new ZeroAlgaePivot(pivot),
            new AlgaePivotToPosition(pivot, 9.3, true),
            new ParallelRaceGroup(
                new FindAlgae(endeffector),
                new AlgaeGrab(endeffector, false)
            ),
            new ParallelCommandGroup(
                new AlgaeGrab(endeffector, true),
                new AlgaePivotGrab(pivot)
            ),
            new HoldPivotPosition(pivot)
        );
    }
}
