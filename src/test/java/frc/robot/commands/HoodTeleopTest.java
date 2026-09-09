package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import frc.robot.subsystems.BallisticShot;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.TargetFinder;
import frc.robot.subsystems.ZoneFinder;
import frc.robot.utilities.ShooterTable;

import java.util.HashSet;
import java.util.Set;

import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class HoodTeleopTest
{
    @Mock
    private ShooterHood mockShooterHood;

    @Mock
    private OI mockOI;

    @Mock
    private TargetFinder mockTargetFinder;

    @Mock
    private ShooterTable mockShooterTable;

    @Mock
    private ZoneFinder mockZoneFinder;

    @Mock
    private BallisticShot mockBallisticShot;

    @Test
    public void testExecute_trenchZoneOverridesEverything()
    {
        HoodTeleop cmd = new HoodTeleop(mockShooterHood, mockOI, mockTargetFinder, mockShooterTable, mockZoneFinder, mockBallisticShot);

        Set<String> trenchZone = new HashSet<>();
        trenchZone.add("TRENCH");
        when(mockZoneFinder.getZones()).thenReturn(trenchZone);
        when(mockOI.getOperatorLeftTrigger()).thenReturn(0.5);

        cmd.execute();
        verify(mockShooterHood).setPosition(ShooterHood.maxPositionRadians);
    }

    @Test
    public void testExecute_leftTriggerWithPassingModeClosest()
    {
        HoodTeleop cmd = new HoodTeleop(mockShooterHood, mockOI, mockTargetFinder, mockShooterTable, mockZoneFinder, mockBallisticShot);

        Set<String> noTrench = new HashSet<>();
        when(mockZoneFinder.getZones()).thenReturn(noTrench);
        when(mockOI.getOperatorLeftTrigger()).thenReturn(0.5);
        when(mockTargetFinder.isPassing()).thenReturn(true);
        when(mockTargetFinder.getRangeToTargetMeters()).thenReturn(Units.inchesToMeters(200));

        cmd.execute();
        verify(mockShooterHood).setPosition(ShooterHood.minPositionRadians - Math.toRadians(10));
    }

    @Test
    public void testExecute_leftTriggerWithPassingModeUnder330()
    {
        HoodTeleop cmd = new HoodTeleop(mockShooterHood, mockOI, mockTargetFinder, mockShooterTable, mockZoneFinder, mockBallisticShot);

        Set<String> noTrench = new HashSet<>();
        when(mockZoneFinder.getZones()).thenReturn(noTrench);
        when(mockOI.getOperatorLeftTrigger()).thenReturn(0.5);
        when(mockTargetFinder.isPassing()).thenReturn(true);
        when(mockTargetFinder.getRangeToTargetMeters()).thenReturn(Units.inchesToMeters(300));

        cmd.execute();
        verify(mockShooterHood).setPosition(ShooterHood.minPositionRadians - Math.toRadians(10));
    }

    @Test
    public void testExecute_leftTriggerWithPassingModeFar()
    {
        HoodTeleop cmd = new HoodTeleop(mockShooterHood, mockOI, mockTargetFinder, mockShooterTable, mockZoneFinder, mockBallisticShot);

        Set<String> noTrench = new HashSet<>();
        when(mockZoneFinder.getZones()).thenReturn(noTrench);
        when(mockOI.getOperatorLeftTrigger()).thenReturn(0.5);
        when(mockTargetFinder.isPassing()).thenReturn(true);
        when(mockTargetFinder.getRangeToTargetMeters()).thenReturn(Units.inchesToMeters(400));

        cmd.execute();
        verify(mockShooterHood).setPosition(ShooterHood.minPositionRadians + Math.toRadians(13));
    }

    @Test
    public void testExecute_leftTriggerWithBallisticShotMode()
    {
        HoodTeleop cmd = new HoodTeleop(mockShooterHood, mockOI, mockTargetFinder, mockShooterTable, mockZoneFinder, mockBallisticShot);

        Set<String> noTrench = new HashSet<>();
        when(mockZoneFinder.getZones()).thenReturn(noTrench);
        when(mockOI.getOperatorLeftTrigger()).thenReturn(0.5);
        when(mockTargetFinder.isPassing()).thenReturn(false);
        when(mockOI.ballisticShotMode()).thenReturn(true);
        when(mockBallisticShot.getShot()).thenReturn(new BallisticShot.Shot(12.5, 0.7, 2.0));

        cmd.execute();
        verify(mockShooterHood).setPosition(0.7);
    }

    @Test
    public void testExecute_leftTriggerWithTableMode()
    {
        HoodTeleop cmd = new HoodTeleop(mockShooterHood, mockOI, mockTargetFinder, mockShooterTable, mockZoneFinder, mockBallisticShot);

        Set<String> noTrench = new HashSet<>();
        when(mockZoneFinder.getZones()).thenReturn(noTrench);
        when(mockOI.getOperatorLeftTrigger()).thenReturn(0.5);
        when(mockTargetFinder.isPassing()).thenReturn(false);
        when(mockOI.ballisticShotMode()).thenReturn(false);
        when(mockTargetFinder.getRangeToTargetMeters()).thenReturn(2.0);
        when(mockShooterTable.getHoodAngle(2.0)).thenReturn(0.6);

        cmd.execute();
        verify(mockShooterHood).setPosition(0.6);
    }

    @Test
    public void testExecute_yButtonCornerShot()
    {
        HoodTeleop cmd = new HoodTeleop(mockShooterHood, mockOI, mockTargetFinder, mockShooterTable, mockZoneFinder, mockBallisticShot);

        Set<String> noTrench = new HashSet<>();
        when(mockZoneFinder.getZones()).thenReturn(noTrench);
        when(mockOI.getOperatorLeftTrigger()).thenReturn(0.0);
        when(mockOI.getOperatorYButton()).thenReturn(true);
        when(mockOI.getOperatorXButton()).thenReturn(false);

        cmd.execute();
        verify(mockShooterHood).setPosition(Math.toRadians(55.0));
    }

    @Test
    public void testExecute_xButtonTowerShot()
    {
        HoodTeleop cmd = new HoodTeleop(mockShooterHood, mockOI, mockTargetFinder, mockShooterTable, mockZoneFinder, mockBallisticShot);

        Set<String> noTrench = new HashSet<>();
        when(mockZoneFinder.getZones()).thenReturn(noTrench);
        when(mockOI.getOperatorLeftTrigger()).thenReturn(0.0);
        when(mockOI.getOperatorYButton()).thenReturn(false);
        when(mockOI.getOperatorXButton()).thenReturn(true);

        cmd.execute();
        verify(mockShooterHood).setPosition(Math.toRadians(61.0));
    }

    @Test
    public void testExecute_noInputsRaisesHood()
    {
        HoodTeleop cmd = new HoodTeleop(mockShooterHood, mockOI, mockTargetFinder, mockShooterTable, mockZoneFinder, mockBallisticShot);

        Set<String> noTrench = new HashSet<>();
        when(mockZoneFinder.getZones()).thenReturn(noTrench);
        when(mockOI.getOperatorLeftTrigger()).thenReturn(0.0);
        when(mockOI.getOperatorYButton()).thenReturn(false);
        when(mockOI.getOperatorXButton()).thenReturn(false);

        cmd.execute();
        verify(mockShooterHood).setPosition(ShooterHood.maxPositionRadians);
    }

    @Test
    public void testEnd_callsStop()
    {
        HoodTeleop cmd = new HoodTeleop(mockShooterHood, mockOI, mockTargetFinder, mockShooterTable, mockZoneFinder, mockBallisticShot);
        cmd.end(false);
        verify(mockShooterHood).stop();
    }

    @Test
    public void testIsFinished_alwaysFalse()
    {
        HoodTeleop cmd = new HoodTeleop(mockShooterHood, mockOI, mockTargetFinder, mockShooterTable, mockZoneFinder, mockBallisticShot);
        assert !cmd.isFinished();
    }
}
