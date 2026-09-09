package frc.robot.commands.Autos;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.Test;
import frc.robot.subsystems.BallisticShot;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.TargetFinder;
import frc.robot.utilities.ShooterTable;

public class TrackFlywheelTest {
    @Test
    public void testExecute_usesBallisticShot_byDefault() {
        Flywheel mockFlywheel = mock(Flywheel.class);
        TargetFinder mockTF = mock(TargetFinder.class);
        ShooterTable mockST = mock(ShooterTable.class);
        BallisticShot mockBS = mock(BallisticShot.class);

        double testSpeed = 9.5;
        when(mockBS.getShot()).thenReturn(new BallisticShot.Shot(testSpeed, 1.0, 1.0));
        when(mockTF.getRangeToTargetMeters()).thenReturn(2.0);

        TrackFlywheel command = new TrackFlywheel(mockFlywheel, mockTF, mockST, mockBS);
        command.execute();

        verify(mockFlywheel).setVelocity(testSpeed);
    }

    @Test
    public void testExecute_usesShooterTable_whenEnabled() {
        Flywheel mockFlywheel = mock(Flywheel.class);
        TargetFinder mockTF = mock(TargetFinder.class);
        ShooterTable mockST = mock(ShooterTable.class);
        BallisticShot mockBS = mock(BallisticShot.class);

        double range = 2.5;
        double tableSpeed = 8.5;
        when(mockTF.getRangeToTargetMeters()).thenReturn(range);
        when(mockST.getFlywheelVelocity(range)).thenReturn(tableSpeed);

        TrackFlywheel command = new TrackFlywheel(mockFlywheel, mockTF, mockST, mockBS, true);
        command.execute();

        verify(mockST).getFlywheelVelocity(range);
        verify(mockFlywheel).setVelocity(tableSpeed);
    }

    @Test
    public void testEnd_callsStop() {
        Flywheel mockFlywheel = mock(Flywheel.class);
        TargetFinder mockTF = mock(TargetFinder.class);
        ShooterTable mockST = mock(ShooterTable.class);
        BallisticShot mockBS = mock(BallisticShot.class);

        TrackFlywheel command = new TrackFlywheel(mockFlywheel, mockTF, mockST, mockBS);
        command.end(false);

        verify(mockFlywheel).stop();
    }

    @Test
    public void testIsFinished_alwaysFalse() {
        Flywheel mockFlywheel = mock(Flywheel.class);
        TargetFinder mockTF = mock(TargetFinder.class);
        ShooterTable mockST = mock(ShooterTable.class);
        BallisticShot mockBS = mock(BallisticShot.class);

        TrackFlywheel command = new TrackFlywheel(mockFlywheel, mockTF, mockST, mockBS);

        assertTrue(!command.isFinished());
    }

    @Test
    public void testConstructor_defaultUsesFalseForBoolean() {
        Flywheel mockFlywheel = mock(Flywheel.class);
        TargetFinder mockTF = mock(TargetFinder.class);
        ShooterTable mockST = mock(ShooterTable.class);
        BallisticShot mockBS = mock(BallisticShot.class);

        when(mockBS.getShot()).thenReturn(new BallisticShot.Shot(8.0, 1.0, 1.0));
        when(mockTF.getRangeToTargetMeters()).thenReturn(2.0);

        TrackFlywheel command = new TrackFlywheel(mockFlywheel, mockTF, mockST, mockBS);
        command.execute();

        // Default should use BallisticShot (false boolean), not ShooterTable
        verify(mockBS).getShot();
    }

    @Test
    public void testConstructor_addsRequirements() {
        Flywheel mockFlywheel = mock(Flywheel.class);
        TargetFinder mockTF = mock(TargetFinder.class);
        ShooterTable mockST = mock(ShooterTable.class);
        BallisticShot mockBS = mock(BallisticShot.class);

        TrackFlywheel command = new TrackFlywheel(mockFlywheel, mockTF, mockST, mockBS);

        assertTrue(command.getRequirements().contains(mockFlywheel));
    }
}
