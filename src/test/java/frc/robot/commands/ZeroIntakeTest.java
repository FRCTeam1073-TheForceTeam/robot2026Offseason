package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.inOrder;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.Test;
import org.mockito.InOrder;
import frc.robot.subsystems.Intake;

public class ZeroIntakeTest {
    private static final double LIMIT_NM = 2.5;

    @Test
    public void testIsFinished_belowThreshold() {
        Intake mockIntake = mock(Intake.class);
        when(mockIntake.getTorqueNm()).thenReturn(2.0);

        ZeroIntake command = new ZeroIntake(mockIntake);

        assertFalse(command.isFinished());
    }

    @Test
    public void testIsFinished_aboveThreshold() {
        Intake mockIntake = mock(Intake.class);
        when(mockIntake.getTorqueNm()).thenReturn(3.0);

        ZeroIntake command = new ZeroIntake(mockIntake);

        assertTrue(command.isFinished());
    }

    @Test
    public void testIsFinished_exactlyAtThreshold_returnsFalse() {
        Intake mockIntake = mock(Intake.class);
        when(mockIntake.getTorqueNm()).thenReturn(LIMIT_NM);

        ZeroIntake command = new ZeroIntake(mockIntake);

        assertFalse(command.isFinished());
    }

    @Test
    public void testExecute_setsVelocity() {
        Intake mockIntake = mock(Intake.class);
        when(mockIntake.getTorqueNm()).thenReturn(0.0);

        ZeroIntake command = new ZeroIntake(mockIntake);
        command.execute();

        verify(mockIntake).setVelocity(-1.0);
    }

    @Test
    public void testEnd_notInterrupted_callsZeroThenStop() {
        Intake mockIntake = mock(Intake.class);

        ZeroIntake command = new ZeroIntake(mockIntake);
        command.end(false);

        InOrder inOrder = inOrder(mockIntake);
        inOrder.verify(mockIntake).zero();
        inOrder.verify(mockIntake).stop();
    }

    @Test
    public void testEnd_interrupted_callsZeroThenStop() {
        Intake mockIntake = mock(Intake.class);

        ZeroIntake command = new ZeroIntake(mockIntake);
        command.end(true);

        InOrder inOrder = inOrder(mockIntake);
        inOrder.verify(mockIntake).zero();
        inOrder.verify(mockIntake).stop();
    }

    @Test
    public void testConstructor_addsRequirements() {
        Intake mockIntake = mock(Intake.class);

        ZeroIntake command = new ZeroIntake(mockIntake);

        assertTrue(command.getRequirements().contains(mockIntake));
    }
}
