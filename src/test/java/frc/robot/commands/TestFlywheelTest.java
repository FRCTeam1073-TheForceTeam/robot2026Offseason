package frc.robot.commands;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InOrder;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.OI;

import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;
import static org.mockito.Mockito.atLeastOnce;

@ExtendWith(MockitoExtension.class)
public class TestFlywheelTest
{
    @Mock
    private Flywheel mockFlywheel;

    @Mock
    private OI mockOI;

    @Test
    public void testInitialize_setsLevelToZero()
    {
        TestFlywheel cmd = new TestFlywheel(mockFlywheel, mockOI);
        cmd.initialize();
        verify(mockFlywheel, times(0)).setVelocity(anyDouble());
    }

    @Test
    public void testExecute_dPadUpEdgeTriggeredIncrementsLevel()
    {
        TestFlywheel cmd = new TestFlywheel(mockFlywheel, mockOI);
        cmd.initialize();

        when(mockOI.getOperatorDPadUp()).thenReturn(true);
        when(mockOI.getOperatorDPadDown()).thenReturn(false);

        cmd.execute();
        verify(mockFlywheel).setVelocity(0.20);

        cmd.execute();
        verify(mockFlywheel, times(2)).setVelocity(0.20);
    }

    @Test
    public void testExecute_holdingDPadUpDoesNotRetrigger()
    {
        TestFlywheel cmd = new TestFlywheel(mockFlywheel, mockOI);
        cmd.initialize();

        when(mockOI.getOperatorDPadUp()).thenReturn(true);
        when(mockOI.getOperatorDPadDown()).thenReturn(false);

        cmd.execute();
        verify(mockFlywheel).setVelocity(0.20);

        cmd.execute();
        verify(mockFlywheel, times(2)).setVelocity(0.20);

        cmd.execute();
        verify(mockFlywheel, times(3)).setVelocity(0.20);
    }

    @Test
    public void testExecute_dPadDownEdgeTriggeredDecrementsLevel()
    {
        TestFlywheel cmd = new TestFlywheel(mockFlywheel, mockOI);
        cmd.initialize();

        when(mockOI.getOperatorDPadUp()).thenReturn(true);
        when(mockOI.getOperatorDPadDown()).thenReturn(false);
        cmd.execute();
        cmd.execute();

        when(mockOI.getOperatorDPadUp()).thenReturn(false);
        when(mockOI.getOperatorDPadDown()).thenReturn(true);
        cmd.execute();

        verify(mockFlywheel).setVelocity(0.0);
    }

    @Test
    public void testExecute_levelClampedToMin()
    {
        TestFlywheel cmd = new TestFlywheel(mockFlywheel, mockOI);
        cmd.initialize();

        when(mockOI.getOperatorDPadUp()).thenReturn(false);
        when(mockOI.getOperatorDPadDown()).thenReturn(true);

        cmd.execute();
        verify(mockFlywheel).setVelocity(0.0);
    }

    @Test
    public void testExecute_levelClampedToMax()
    {
        TestFlywheel cmd = new TestFlywheel(mockFlywheel, mockOI);
        cmd.initialize();

        when(mockOI.getOperatorDPadDown()).thenReturn(false);

        for (int i = 0; i < 121; i++) {
            when(mockOI.getOperatorDPadUp()).thenReturn(true);
            cmd.execute();
            when(mockOI.getOperatorDPadUp()).thenReturn(false);
            cmd.execute();
        }

        verify(mockFlywheel, atLeastOnce()).setVelocity(120 * 0.20);
    }

    @Test
    public void testEnd_callsStop()
    {
        TestFlywheel cmd = new TestFlywheel(mockFlywheel, mockOI);

        InOrder inOrder = inOrder(mockFlywheel);
        cmd.end(false);
        inOrder.verify(mockFlywheel).stop();
    }

    @Test
    public void testIsFinished_alwaysFalse()
    {
        TestFlywheel cmd = new TestFlywheel(mockFlywheel, mockOI);
        assert !cmd.isFinished();
    }
}
