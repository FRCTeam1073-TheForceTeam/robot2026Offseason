// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static java.util.Map.entry;

import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.TreeSet;

/**
 * Single source of truth for which autonomous selections drive a routine and which Choreo
 * trajectory file each one uses. Both the trajectory preload and the autonomous command
 * builder read from here, so the two can't drift apart.
 */
public final class AutoCatalog
{
    /** Selections that run an autonomous routine (shooting, intake, and driving). */
    private static final Set<String> ROUTINE_AUTOS = Set.of(
            RobotContainer.centerDepotOutpost,
            RobotContainer.centerDepotClimb,
            RobotContainer.rightBumpSteal,
            RobotContainer.rightTrenchHalfOutpost,
            RobotContainer.rightTrenchHalfDouble,
            RobotContainer.rightTrenchHalfDoubleBump,
            RobotContainer.leftTrenchHalfDouble,
            RobotContainer.leftTrenchHalfDoubleBump,
            RobotContainer.leftBumpFull,
            RobotContainer.basicAuto,
            RobotContainer.rightBumpFollow,
            RobotContainer.leftBumpFollow);

    /** Routine selections that drive along a Choreo trajectory, mapped to the .traj name. */
    private static final Map<String, String> TRAJECTORY_FILES = Map.ofEntries(
            entry(RobotContainer.centerDepotOutpost, "CenterDepotOutpost"),
            entry(RobotContainer.centerDepotClimb, "CenterDepotClimb"),
            entry(RobotContainer.rightBumpSteal, "RightBumpSteal"),
            entry(RobotContainer.rightTrenchHalfOutpost, "RightTrenchHalfOutpost"),
            entry(RobotContainer.rightTrenchHalfDouble, "RightTrenchHalfDouble"),
            entry(RobotContainer.rightTrenchHalfDoubleBump, "RightTrenchHalfDoubleBump"),
            entry(RobotContainer.leftTrenchHalfDouble, "LeftTrenchHalfDouble"),
            entry(RobotContainer.leftTrenchHalfDoubleBump, "LeftTrenchHalfDoubleBump"),
            entry(RobotContainer.leftBumpFull, "LeftBumpFull"),
            entry(RobotContainer.rightBumpFollow, "RightBumpFollow"),
            entry(RobotContainer.leftBumpFollow, "LeftBumpFollow"));

    private AutoCatalog()
    {
    }

    /** True if this selection runs a routine, which means the command builder is used. */
    public static boolean runsRoutine(String selection)
    {
        return ROUTINE_AUTOS.contains(selection);
    }

    /**
     * The Choreo trajectory name (no extension) for a selection, or empty if the selection
     * does not drive along a trajectory.
     */
    public static Optional<String> trajectoryFile(String selection)
    {
        return Optional.ofNullable(TRAJECTORY_FILES.get(selection));
    }

    /** Every distinct trajectory name that needs to be preloaded, in a stable order. */
    public static Set<String> trajectoryFiles()
    {
        return new TreeSet<>(TRAJECTORY_FILES.values());
    }

    /** Routine selections that run with no trajectory file (currently only basicAuto). */
    public static Set<String> routinesWithoutTrajectory()
    {
        Set<String> result = new TreeSet<>(ROUTINE_AUTOS);
        result.removeAll(TRAJECTORY_FILES.keySet());
        return result;
    }
}
