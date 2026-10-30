package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Set;

import org.junit.jupiter.api.Test;

public class AutoCatalogTest {
    @Test
    public void testOnlyBasicAutoRunsWithoutTrajectory() {
        // basicAuto is a routine with no .traj file. Any new exception should be added on purpose.
        assertEquals(Set.of(RobotContainer.basicAuto), AutoCatalog.routinesWithoutTrajectory());
    }

    @Test
    public void testFollowAutosUseTheirOwnTrajectories() {
        assertEquals("RightBumpFollow", AutoCatalog.trajectoryFile(RobotContainer.rightBumpFollow).orElseThrow());
        assertEquals("LeftBumpFollow", AutoCatalog.trajectoryFile(RobotContainer.leftBumpFollow).orElseThrow());
    }

    @Test
    public void testNonRoutineSelectionsHaveNoTrajectory() {
        for (String selection : new String[] {RobotContainer.noLevelAuto, RobotContainer.startLine,
                    RobotContainer.centerHub, RobotContainer.basicTest}) {
            assertFalse(AutoCatalog.runsRoutine(selection), selection + " should not run a routine");
            assertTrue(AutoCatalog.trajectoryFile(selection).isEmpty(), selection + " should have no trajectory");
        }
    }
}
