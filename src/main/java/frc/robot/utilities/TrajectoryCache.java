// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.Map;
import java.util.Optional;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Function;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

/**
 * Holds loaded Choreo trajectories by name. Entries are loaded at most once, so the preload
 * thread and the main thread can both ask for a trajectory without loading it twice. A
 * failed load is cached as empty so it is not retried every loop.
 */
public class TrajectoryCache
{
    private final Map<String, Optional<Trajectory<SwerveSample>>> cache = new ConcurrentHashMap<>();
    private final Function<String, Optional<Trajectory<SwerveSample>>> loader;

    public TrajectoryCache(Function<String, Optional<Trajectory<SwerveSample>>> loader)
    {
        this.loader = loader;
    }

    /** Returns the trajectory, loading it now (blocking) if it has not been loaded yet. */
    public Optional<Trajectory<SwerveSample>> get(String name)
    {
        return cache.computeIfAbsent(name, this::load);
    }

    /** Non-blocking check: true only if the trajectory is already loaded and valid. */
    public boolean isReady(String name)
    {
        Optional<Trajectory<SwerveSample>> entry = cache.get(name);
        return entry != null && entry.isPresent();
    }

    private Optional<Trajectory<SwerveSample>> load(String name)
    {
        try {
            Optional<Trajectory<SwerveSample>> traj = loader.apply(name);
            if (traj.isEmpty()) {
                System.err.println("TRAJECTORY NOT FOUND: " + name);
            }
            return traj;
        } catch (RuntimeException e) {
            System.err.println("TRAJECTORY LOAD THREW EXCEPTION (" + name + "): " + e.getMessage());
            return Optional.empty();
        }
    }
}
