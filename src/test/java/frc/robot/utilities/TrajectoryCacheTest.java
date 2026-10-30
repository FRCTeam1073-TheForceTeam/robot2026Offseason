package frc.robot.utilities;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.Test;

public class TrajectoryCacheTest {
    @Test
    public void testMissingTrajectoryIsEmptyAndNotReady() {
        TrajectoryCache cache = new TrajectoryCache(name -> Optional.empty());

        assertTrue(cache.get("Nope").isEmpty());
        assertFalse(cache.isReady("Nope"));
    }

    @Test
    public void testFailedLoadIsNotRetried() {
        List<String> calls = new ArrayList<>();
        TrajectoryCache cache = new TrajectoryCache(name -> {
                    calls.add(name);
                    throw new RuntimeException("bad file");
        });

        assertTrue(cache.get("Broken").isEmpty());
        assertTrue(cache.get("Broken").isEmpty());
        assertFalse(cache.isReady("Broken"));
        assertEquals(1, calls.size(), "a failed load should be remembered, not retried every loop");
    }

    @Test
    public void testIsReadyDoesNotLoad() {
        List<String> calls = new ArrayList<>();
        TrajectoryCache cache = new TrajectoryCache(name -> {
                    calls.add(name);
                    return Optional.empty();
        });

        // The disabled loop must not block on file I/O, so isReady only looks at what is loaded.
        assertFalse(cache.isReady("Lazy"));
        assertTrue(calls.isEmpty());
    }

    @Test
    public void testEachNameIsLoadedOnce() {
        List<String> calls = new ArrayList<>();
        TrajectoryCache cache = new TrajectoryCache(name -> {
                    calls.add(name);
                    return Optional.empty();
        });

        cache.get("A");
        cache.get("A");
        cache.get("B");

        assertEquals(List.of("A", "B"), calls);
    }
}
