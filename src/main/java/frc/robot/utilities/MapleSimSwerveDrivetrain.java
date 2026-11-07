package frc.robot.utilities;

import edu.wpi.first.math.system.plant.DCMotor;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class MapleSimSwerveDrivetrain {
    // Create and configure a drivetrain simulation configuration
    final DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
            // Specify gyro type (for realistic gyro drifting and error simulation)
            .withGyro(COTS.ofPigeon2())
            // Specify swerve module (for realistic swerve dynamics)
            .withSwerveModule(COTS.ofMark4(
                    DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
                    DCMotor.getFalcon500(1), // Steer motor is a Falcon 500
                    COTS.WHEELS.COLSONS.cof, // Use the COF for Colson Wheels
                    3)) // L3 Gear ratio
            // Configures the track length and track width (spacing between swerve modules)
            .withTrackLengthTrackWidth(Inches.of(24), Inches.of(24))
            // Configures the bumper size (dimensions of the robot bumper)
            .withBumperSize(Inches.of(30), Inches.of(30));
}
