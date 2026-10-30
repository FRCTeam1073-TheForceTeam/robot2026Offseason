package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class MathUtils
{
    public static double wrapAngleRadians(double angle)
    {
        angle = (angle + Math.PI) % (2 * Math.PI);
        if (angle < 0)
        {
            angle += 2 * Math.PI;
        }
        return angle - Math.PI;
    }

    public static double wrapAngleDegrees(double angle)
    {
        angle = (angle + 180) % (360);
        if (angle < 0)
        {
            angle += 360;
        }
        return angle - 180;
    }

    public static double motorTurnsToRadians(double motorTurns, double gearRatio)
    {
        return motorTurns * 2.0 * Math.PI / gearRatio;
    }

    public static double radiansToMotorTurns(double radians, double gearRatio)
    {
        return radians * gearRatio / (2.0 * Math.PI);
    }

    public static double motorTurnsToMeters(double motorTurns, double turnsPerMeter, double gearRatio)
    {
        return motorTurns / (turnsPerMeter * gearRatio);
    }

    public static double metersToMotorTurns(double meters, double turnsPerMeter, double gearRatio)
    {
        return meters * turnsPerMeter * gearRatio;
    }

    public static double currentToForce(double amps, double ampsPerUnit)
    {
        return amps / ampsPerUnit;
    }

    public static boolean withinTolerance(double target, double current, double tolerance)
    {
        return Math.abs(target - current) < tolerance;
    }

    public static double clampThenLimit(SlewRateLimiter limiter, double target, double min, double max)
    {
        return limiter.calculate(MathUtil.clamp(target, min, max));
    }

    public static double limitThenClamp(SlewRateLimiter limiter, double target, double min, double max)
    {
        return MathUtil.clamp(limiter.calculate(target), min, max);
    }
}
