// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

/**
 * Work that is safe to run off the main thread and that removes first-use cost from autonomous:
 * loading the classes auto needs, and running the trajectory sampling code once.
 */
public final class TrajectoryWarmup
{
  // Classes that are first touched when auto starts. Initializing them here moves the cost into
  // the disabled period.
  private static final String[] CLASSES = {
      "choreo.Choreo",
      "choreo.trajectory.Trajectory",
      "choreo.trajectory.SwerveSample",
      "choreo.trajectory.EventMarker",
      "frc.robot.commands.DrivePath",
      "frc.robot.commands.AutoRunner",
  };

  private static final int SAMPLES_PER_TRAJECTORY = 20;

  private TrajectoryWarmup()
  {
  }

  public static void loadClasses()
  {
    for (String name : CLASSES) {
      try {
        Class.forName(name, true, TrajectoryWarmup.class.getClassLoader());
      } catch (ClassNotFoundException | LinkageError e) {
        System.err.println("AUTO WARMUP could not load " + name + ": " + e);
      }
    }
  }

  /** Samples the trajectory for both alliances and every split, discarding the results. */
  public static void touchSamples(Trajectory<SwerveSample> traj)
  {
    double total = traj.getTotalTime();
    for (boolean red : new boolean[] {false, true}) {
      traj.getInitialPose(red);
      for (int i = 0; i <= SAMPLES_PER_TRAJECTORY; i++) {
        traj.sampleAt(total * i / SAMPLES_PER_TRAJECTORY, red).ifPresent(sample -> sample.getChassisSpeeds());
      }
    }
    for (int s = 0; s < traj.splits().size(); s++) {
      traj.getSplit(s).ifPresent(TrajectoryWarmup::touchSplit);
    }
  }

  private static void touchSplit(Trajectory<SwerveSample> split)
  {
    split.getInitialPose(false);
    split.sampleAt(split.getTotalTime(), false);
  }
}
