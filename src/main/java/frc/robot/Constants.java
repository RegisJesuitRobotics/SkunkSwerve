package frc.robot;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.SwerveModule.SwerveModuleConfiguration;
import frc.robot.subsystems.swerve.SwerveModule.SharedSwerveModuleConfiguration;
import frc.robot.utils.TunablePIDFFFGains;
import frc.robot.utils.TunablePIDGains;

/**
 * File containing all constants for the robot.
 */
public final class Constants {
    private Constants() {}

    public static class DriveTrainConstants {
        private DriveTrainConstants() {}

        public static final int NUM_MODULES = 4;
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
        public static final double DRIVE_GEAR_REDUCTION = (50.0 / 14) * (17.0 / 27) * (45.0 / 15);

        public static final double STEER_GEAR_REDUCTION = 150.0 / 7.0;

        public static final boolean INVERT_GYRO = false;

        public static final double DRIVE_PEAK_CURRENT_LIMIT = 65.0;
        public static final double DRIVE_CONTINUOUS_CURRENT_LIMIT = 35.0;
        public static final double DRIVE_CONTINUOUS_CURRENT_LIMIT_TIME_SECONDS = 0.2;
        public static final double STEER_PEAK_CURRENT_LIMIT = 45.0;
        public static final double STEER_CONTINUOUS_CURRENT_LIMIT = 25.0;
        public static final double STEER_CONTINUOUS_CURRENT_LIMIT_TIME_SECONDS = 0.2;

        public static final double NOMINAL_VOLTAGE = 12.0;

        // For talons PID full output is 1023 except for all FFF gains
        // TODO: TUNE
        public static final TunablePIDFFFGains DRIVE_VELOCITY_GAINS = new TunablePIDFFFGains(
                "gains/DriveVelocity", 0.02, 0.0, 0.0, 0.6712106209979143, 2.019606167307655, 0.089374
        );

        // TODO: tune
        public static final TunablePIDGains STEER_POSITION_GAINS = new TunablePIDGains(
                "gains/SteerPosition", 0.2, 0.0, 0.1
        );

        // Left right distance between center of wheels
        public static final double TRACKWIDTH_METERS = Units.inchesToMeters(24.75);

        // Front back distance between center of wheels
        public static final double WHEELBASE_METERS = Units.inchesToMeters(24.75);

        public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
                new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
                new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
                new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
                new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0) };

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(MODULE_TRANSLATIONS);

        public static final double MOTOR_FREE_SPEED_RPM = 6380.0;
        public static final double MAX_VELOCITY_METERS_PER_SECOND = (MOTOR_FREE_SPEED_RPM * WHEEL_DIAMETER_METERS
                * Math.PI) / (60.0 * DRIVE_GEAR_REDUCTION);

        public static final double MAX_PATH_ACCELERATION_METERS_PER_SECOND_SQUARED = MAX_VELOCITY_METERS_PER_SECOND
                / 4.0;

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = Math.PI * 2;

        public static final double TRANSLATION_RATE_LIMIT_METERS_SECOND = 10.0;
        public static final double ANGULAR_RATE_LIMIT_RADIANS_SECOND = 5.0 * Math.PI;
        public static final double TELEOP_MINIMUM_VELOCITY_METERS_PER_SECOND = 0.25;

        public static final double ANGLE_TOLERANCE_RADIANS = Units.degreesToRadians(0.25);
        public static final double VELOCITY_TOLERANCE_METERS_PER_SECOND = 0.05;

        // For pose estimation. Increase to trust model LESS
        public static final Vector<N3> STATE_STD_DEVS = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
        public static final Vector<N1> LOCAL_MEASUREMENT_STD_DEVS = VecBuilder.fill(Units.degreesToRadians(0.01));
        public static final Vector<N3> VISION_STD_DEVS = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.01));

        private static final SharedSwerveModuleConfiguration SHARED_SWERVE_MODULE_CONFIGURATION = new SharedSwerveModuleConfiguration(
                DRIVE_GEAR_REDUCTION, STEER_GEAR_REDUCTION, DRIVE_PEAK_CURRENT_LIMIT, DRIVE_CONTINUOUS_CURRENT_LIMIT,
                DRIVE_CONTINUOUS_CURRENT_LIMIT_TIME_SECONDS, STEER_PEAK_CURRENT_LIMIT, STEER_CONTINUOUS_CURRENT_LIMIT,
                STEER_CONTINUOUS_CURRENT_LIMIT_TIME_SECONDS, NOMINAL_VOLTAGE, WHEEL_DIAMETER_METERS,
                MAX_VELOCITY_METERS_PER_SECOND, DRIVE_VELOCITY_GAINS, STEER_POSITION_GAINS, ANGLE_TOLERANCE_RADIANS
        );

        public static final SwerveModuleConfiguration FRONT_LEFT_MODULE_CONFIGURATION = new SwerveModuleConfiguration(
                1, 5, 9, true, true, -67.8, false, SHARED_SWERVE_MODULE_CONFIGURATION
        );

        public static final SwerveModuleConfiguration FRONT_RIGHT_MODULE_CONFIGURATION = new SwerveModuleConfiguration(
                2, 6, 10, true, true, 75.05, false, SHARED_SWERVE_MODULE_CONFIGURATION
        );

        public static final SwerveModuleConfiguration BACK_LEFT_MODULE_CONFIGURATION = new SwerveModuleConfiguration(
                3, 7, 11, true, true, 78.84, false, SHARED_SWERVE_MODULE_CONFIGURATION
        );

        public static final SwerveModuleConfiguration BACK_RIGHT_MODULE_CONFIGURATION = new SwerveModuleConfiguration(
                4, 8, 12, true, true, -156.8, false, SHARED_SWERVE_MODULE_CONFIGURATION
        );
    }

    public static class AutoConstants {
        // TODO: tune
        public static final TunablePIDGains PATH_TRANSLATION_POSITION_GAINS = new TunablePIDGains(
                "gains/PathXY", 0.5, 0.0, 0.0
        );
        public static final TunablePIDGains PATH_ANGULAR_POSITION_GAINS = new TunablePIDGains(
                "gains/PathAngular", 1.0, 0.0, 0.0
        );
        public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
                DriveTrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
                DriveTrainConstants.MAX_PATH_ACCELERATION_METERS_PER_SECOND_SQUARED
        );
    }

    public static class MiscConstants {
        private MiscConstants() {}

        public static final int[] usedControllerPorts = { 0 };
        public static final boolean enablePathPlannerServer = true;
    }
}
