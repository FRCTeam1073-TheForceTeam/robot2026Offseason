// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class Drivetrain extends SubsystemBase {
    private final CommandSwerveDrivetrain ctre;
    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    private boolean parkingBrakeOn = false;

    public Drivetrain() {
        super.setSubsystem("Drivetrain");

        ctre = new CommandSwerveDrivetrain(
            TunerConstants.DrivetrainConstants,
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight
        );

        // Configure logging (optional, for Tuner X SysId)
        SignalLogger.setPath("/media/sda1/ctre-logs/");
    }

    public double getGyroHeadingDegrees() {
        return ctre.getPigeon2().getYaw().refresh().getValueAsDouble();
    }

    public double getGyroHeadingRadians() {
        return getGyroHeadingDegrees() * Math.PI / 180.0;
    }

    public Rotation2d getGyroHeading() {
        return ctre.getPigeon2().getRotation2d();
    }

    public double getWrappedGyroHeadingDegrees() {
        return MathUtils.wrapAngleDegrees(getGyroHeadingDegrees());
    }

    public double getWrappedGyroHeadingRadians() {
        return MathUtils.wrapAngleRadians(getGyroHeadingDegrees() * Math.PI / 180);
    }

    public double getPitch() {
        return ctre.getPigeon2().getPitch().getValueAsDouble();
    }

    public double getRoll() {
        return ctre.getPigeon2().getRoll().getValueAsDouble();
    }

    public void zeroHeading() {
        ctre.getPigeon2().setYaw(0);
    }

    public void setTargetChassisSpeeds(ChassisSpeeds speeds) {
        // Don't apply drive commands if parking brake is active
        if (!parkingBrakeOn) {
            ctre.setControl(applyRobotSpeeds.withSpeeds(speeds));
        }
    }

    public ChassisSpeeds getChassisSpeeds() {
        return ctre.getState().Speeds;
    }

    public void setBrakes(boolean brakeOn) {
        ctre.configNeutralMode(brakeOn ?
            com.ctre.phoenix6.signals.NeutralModeValue.Brake :
            com.ctre.phoenix6.signals.NeutralModeValue.Coast);
    }

    public SwerveDriveKinematics getKinematics() {
        return ctre.getKinematics();
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return ctre.getState().ModulePositions;
    }

    public void resetOdometry(Pose2d where) {
        ctre.resetPose(where);
    }

    public Pose2d getOdometry() {
        return ctre.getState().Pose;
    }

    public double getOdometryX() {
        return ctre.getState().Pose.getX();
    }

    public double getOdometryY() {
        return ctre.getState().Pose.getY();
    }

    public double getOdometryThetaRadians() {
        return ctre.getState().Pose.getRotation().getRadians();
    }

    public Pose3d get3dOdometry() {
        Pose2d odo = getOdometry();
        return new Pose3d(odo.getX(), odo.getY(), 0.0, new Rotation3d(getRoll(), getPitch(), getOdometryThetaRadians()));
    }

    public void parkingBrake(boolean enable) {
        System.err.println("DEBUG Drivetrain.parkingBrake: enable=" + enable);
        parkingBrakeOn = enable;
        System.err.println("DEBUG: Parking brake state set to " + parkingBrakeOn);
    }

    public boolean getParkingBrake() {
        return parkingBrakeOn;
    }

    public double getAverageLoad() {
        double totalLoad = 0;
        for (int i = 0; i < 4; i++) {
            totalLoad += ctre.getModule(i).getDriveMotor().getTorqueCurrent(true).getValueAsDouble();
        }
        return totalLoad / 4.0;
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp, double[] stddevs) {
        ctre.addVisionMeasurement(pose, timestamp, VecBuilder.fill(stddevs[0], stddevs[1], stddevs[2]));
    }

    public void resetRotation(Rotation2d rotation) {
        ctre.resetRotation(rotation);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("ParkingBrake", this::getParkingBrake, null);
        builder.addDoubleProperty("Odo X", this::getOdometryX, null);
        builder.addDoubleProperty("Odo Y", this::getOdometryY, null);
        builder.addDoubleProperty("Odo Theta(RAD)", this::getOdometryThetaRadians, null);
        builder.addDoubleProperty("Odo Gyro Heading(DEG)", this::getGyroHeadingDegrees, null);
        builder.addDoubleProperty("Odo Gyro Wrapped Heading", this::getWrappedGyroHeadingDegrees, null);
        builder.addDoubleProperty("Pitch", this::getPitch, null);
        builder.addDoubleProperty("Roll", this::getRoll, null);
    }

    @Override
    public void periodic() {
        // Apply parking brake if active (must be done every loop)
        if (parkingBrakeOn) {
            System.err.println("DEBUG periodic: Applying parking brake");
            // Lock wheels in X pattern (±45° on each corner)
            // FL: +45°, FR: -45°, BL: -45°, BR: +45°
            ctre.getModule(0).apply(
                new com.ctre.phoenix6.controls.PositionVoltage(Math.PI/4),  // 45° in radians
                new com.ctre.phoenix6.controls.NeutralOut());                // 0 drive velocity
            ctre.getModule(1).apply(
                new com.ctre.phoenix6.controls.PositionVoltage(-Math.PI/4), // -45° in radians
                new com.ctre.phoenix6.controls.NeutralOut());
            ctre.getModule(2).apply(
                new com.ctre.phoenix6.controls.PositionVoltage(-Math.PI/4), // -45° in radians
                new com.ctre.phoenix6.controls.NeutralOut());
            ctre.getModule(3).apply(
                new com.ctre.phoenix6.controls.PositionVoltage(Math.PI/4),  // 45° in radians
                new com.ctre.phoenix6.controls.NeutralOut());
        }

        var state = ctre.getStateCopy();

        // Log drivetrain-level telemetry
        Logger.recordOutput("Drivetrain/Pose", state.Pose);
        Logger.recordOutput("Drivetrain/Speeds", state.Speeds);
        Logger.recordOutput("Drivetrain/ModuleStates", state.ModuleStates);
        Logger.recordOutput("Drivetrain/ModuleTargets", state.ModuleTargets);
        Logger.recordOutput("Drivetrain/OdometryPeriod", state.OdometryPeriod);
        Logger.recordOutput("Drivetrain/SuccessfulDAQs", state.SuccessfulDaqs);
        Logger.recordOutput("Drivetrain/FailedDAQs", state.FailedDaqs);

        // Log module offsets for debugging (verify CANcoder calibration)
        SmartDashboard.putNumber("Drivetrain/Module0/MagnetOffset", ctre.getModule(0).getEncoder().getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Drivetrain/Module1/MagnetOffset", ctre.getModule(1).getEncoder().getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Drivetrain/Module2/MagnetOffset", ctre.getModule(2).getEncoder().getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Drivetrain/Module3/MagnetOffset", ctre.getModule(3).getEncoder().getAbsolutePosition().getValueAsDouble());

        // Log current module angles (for parking brake debugging)
        var moduleStates = ctre.getState().ModuleStates;
        SmartDashboard.putNumber("Drivetrain/Module0/SteerAngle", moduleStates[0].angle.getDegrees());
        SmartDashboard.putNumber("Drivetrain/Module1/SteerAngle", moduleStates[1].angle.getDegrees());
        SmartDashboard.putNumber("Drivetrain/Module2/SteerAngle", moduleStates[2].angle.getDegrees());
        SmartDashboard.putNumber("Drivetrain/Module3/SteerAngle", moduleStates[3].angle.getDegrees());
    }
}
