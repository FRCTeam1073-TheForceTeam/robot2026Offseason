package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.CoralEndeffector;
import frc.robot.subsystems.Drivetrain;

public class RemoveAlgae extends Command
{
    CoralElevator elevator;
    CoralEndeffector endEffector;
    Drivetrain drivetrain;
    int level;

    double targetHeight = 0;

    public RemoveAlgae(CoralElevator elevator, CoralEndeffector endEffector, Drivetrain drivetrain, int level)
    {
        this.elevator = elevator;
        this.endEffector = endEffector;
        this.drivetrain = drivetrain;
        this.level = level;

        if (level == 2)
        {
            targetHeight = 17.3;
        }
        else if (level == 3)
        {
            targetHeight = 28.2;
        }

        addRequirements(elevator, endEffector, drivetrain);
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute()
    {
        if (!(Math.abs(elevator.getPosition() - targetHeight) < (0.01 * targetHeight)))
        {
            elevator.setPosition(targetHeight);
        }
        endEffector.setVelocity(-20);
        if (Math.abs(elevator.getPosition() - targetHeight) < (0.01 * targetHeight))
        {
            elevator.setPosition(targetHeight + 10);
            drivetrain.setTargetChassisSpeeds(new ChassisSpeeds(-0.5, 0, 0));
        }
    }

    @Override 
    public void end(boolean interrupted)
    {
        drivetrain.setTargetChassisSpeeds(new ChassisSpeeds(0, 0, 0));
        endEffector.setVelocity(0);
        elevator.setVelocity(0);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
