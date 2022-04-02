// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.SwerveModule.SwerveModuleConfiguration;
import frc.robot.utils.PIDFGains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    private Constants() {}

    public static class DriveTrainConstants {
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
        public static final double DRIVE_GEAR_REDUCTION = 6.0;

        public static final double STEERING_GEAR_REDUCTION = 3.0;

        public static final double DRIVE_CURRENT_LIMIT = 40.0;
        public static final double STEER_CURRENT_LIMIT = 20.0;

        public static final double NOMINAL_VOLTAGE = 12.0;

        public static final PIDFGains DRIVE_VELOCITY_GAINS = new PIDFGains(0.0, 0.0, 0.0, 0.0469, 0.0);
        public static final PIDFGains STEER_POSITION_GAINS = new PIDFGains(0.0, 0.0, 0.0, 0.0, 0.0);

        public static final SwerveModuleConfiguration FRONT_LEFT_MODULE_CONFIGURATION = new SwerveModuleConfiguration(1,
                5, 9, DRIVE_GEAR_REDUCTION, STEERING_GEAR_REDUCTION, DRIVE_CURRENT_LIMIT, STEER_CURRENT_LIMIT, false,
                false, NOMINAL_VOLTAGE, 0.0, false, WHEEL_DIAMETER_METERS, DRIVE_VELOCITY_GAINS, STEER_POSITION_GAINS);

        public static final SwerveModuleConfiguration FRONT_RIGHT_MODULE_CONFIGURATION = new SwerveModuleConfiguration(
                2, 6, 10, DRIVE_GEAR_REDUCTION, STEERING_GEAR_REDUCTION, DRIVE_CURRENT_LIMIT, STEER_CURRENT_LIMIT,
                false, false, NOMINAL_VOLTAGE, 0.0, false, WHEEL_DIAMETER_METERS, DRIVE_VELOCITY_GAINS,
                STEER_POSITION_GAINS);

        public static final SwerveModuleConfiguration BACK_LEFT_MODULE_CONFIGURATION = new SwerveModuleConfiguration(3,
                7, 11, DRIVE_GEAR_REDUCTION, STEERING_GEAR_REDUCTION, DRIVE_CURRENT_LIMIT, STEER_CURRENT_LIMIT, false,
                false, NOMINAL_VOLTAGE, 0.0, false, WHEEL_DIAMETER_METERS, DRIVE_VELOCITY_GAINS, STEER_POSITION_GAINS);

        public static final SwerveModuleConfiguration BACK_RIGHT_MODULE_CONFIGURATION = new SwerveModuleConfiguration(4,
                8, 12, DRIVE_GEAR_REDUCTION, STEERING_GEAR_REDUCTION, DRIVE_CURRENT_LIMIT, STEER_CURRENT_LIMIT, false,
                false, NOMINAL_VOLTAGE, 0.0, false, WHEEL_DIAMETER_METERS, DRIVE_VELOCITY_GAINS, STEER_POSITION_GAINS);

        // Left right distance between center of wheels
        public static final double TRACKWIDTH_METERS = Units.inchesToMeters(30);

        // Front back distance between center of wheels
        public static final double WHEELBASE_METERS = Units.inchesToMeters(30);

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
                new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
                new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
                new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0));

        public static final double MOTOR_FREE_SPEED_RPM = 6380.0;
        public static final double MAX_VELOCITY_METERS_PER_SECOND = (MOTOR_FREE_SPEED_RPM * DRIVE_GEAR_REDUCTION
                * WHEEL_DIAMETER_METERS * Math.PI) / (60.0);
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = (MAX_VELOCITY_METERS_PER_SECOND)
                / (Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0));

        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = MAX_VELOCITY_METERS_PER_SECOND;

        public static final double MAX_TELEOP_VELOCITY_METERS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND * 0.75;
        public static final double MAX_TELEOP_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
                * 0.75;

        public static final double PATH_POSITIONAL_VELOCITY_P = 1.0;
        public static final double PATH_ANGULAR_VELOCITY_P = 1.0;
        public static final Constraints ANGULAR_CONSTRAINTS = new Constraints(MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

        public static final double ANGLE_TOLERANCE_RADIANS = Units.degreesToRadians(0.5);
        public static final double VELOCITY_TOLERANCE_METERS_PER_SECOND = 0.05;

    }
}
