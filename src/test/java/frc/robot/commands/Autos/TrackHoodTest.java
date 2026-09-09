package frc.robot.commands.Autos;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.Test;
import frc.robot.subsystems.BallisticShot;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.TargetFinder;
import frc.robot.utilities.ShooterTable;

public class TrackHoodTest {
    @Test
    public void testExecute_usesBallisticShot_byDefault() {
        ShooterHood mockHood = mock(ShooterHood.class);
        TargetFinder mockTF = mock(TargetFinder.class);
        ShooterTable mockST = mock(ShooterTable.class);
        BallisticShot mockBS = mock(BallisticShot.class);

        double testAngle = 1.234;
        when(mockBS.getShot()).thenReturn(new BallisticShot.Shot(8.0, testAngle, 1.0));
        when(mockTF.getRangeToTargetMeters()).thenReturn(2.0);

        TrackHood command = new TrackHood(mockHood, mockTF, mockST, mockBS);
        command.execute();

        verify(mockHood).setPosition(testAngle);
    }

    @Test
    public void testExecute_usesShooterTable_whenEnabled() {
        ShooterHood mockHood = mock(ShooterHood.class);
        TargetFinder mockTF = mock(TargetFinder.class);
        ShooterTable mockST = mock(ShooterTable.class);
        BallisticShot mockBS = mock(BallisticShot.class);

        double range = 2.5;
        double tableAngle = Math.toRadians(60.0);
        when(mockTF.getRangeToTargetMeters()).thenReturn(range);
        when(mockST.getHoodAngle(range)).thenReturn(tableAngle);

        TrackHood command = new TrackHood(mockHood, mockTF, mockST, mockBS, true);
        command.execute();

        verify(mockST).getHoodAngle(range);
        verify(mockHood).setPosition(tableAngle);
    }

    @Test
    public void testEnd_callsStop() {
        ShooterHood mockHood = mock(ShooterHood.class);
        TargetFinder mockTF = mock(TargetFinder.class);
        ShooterTable mockST = mock(ShooterTable.class);
        BallisticShot mockBS = mock(BallisticShot.class);

        TrackHood command = new TrackHood(mockHood, mockTF, mockST, mockBS);
        command.end(false);

        verify(mockHood).stop();
    }

    @Test
    public void testIsFinished_alwaysFalse() {
        ShooterHood mockHood = mock(ShooterHood.class);
        TargetFinder mockTF = mock(TargetFinder.class);
        ShooterTable mockST = mock(ShooterTable.class);
        BallisticShot mockBS = mock(BallisticShot.class);

        TrackHood command = new TrackHood(mockHood, mockTF, mockST, mockBS);

        assertTrue(!command.isFinished());
    }

    @Test
    public void testConstructor_defaultUsesFalseForBoolean() {
        ShooterHood mockHood = mock(ShooterHood.class);
        TargetFinder mockTF = mock(TargetFinder.class);
        ShooterTable mockST = mock(ShooterTable.class);
        BallisticShot mockBS = mock(BallisticShot.class);

        when(mockBS.getShot()).thenReturn(new BallisticShot.Shot(8.0, 1.0, 1.0));
        when(mockTF.getRangeToTargetMeters()).thenReturn(2.0);

        TrackHood command = new TrackHood(mockHood, mockTF, mockST, mockBS);
        command.execute();

        // Default should use BallisticShot (false boolean), not ShooterTable
        verify(mockBS).getShot();
    }

    @Test
    public void testConstructor_addsRequirements() {
        ShooterHood mockHood = mock(ShooterHood.class);
        TargetFinder mockTF = mock(TargetFinder.class);
        ShooterTable mockST = mock(ShooterTable.class);
        BallisticShot mockBS = mock(BallisticShot.class);

        TrackHood command = new TrackHood(mockHood, mockTF, mockST, mockBS);

        assertTrue(command.getRequirements().contains(mockHood));
    }
}
