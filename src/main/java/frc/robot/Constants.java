// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.SwerveModule.SwerveModuleConfiguration;
import frc.robot.subsystems.swerve.SwerveModule.SwerveModuleConfiguration.SharedSwerveModuleConfiguration;
import frc.robot.utils.PIDFFFGains;
import frc.robot.utils.PIDGains;

public final class Constants {
    private Constants() {}

    public static class DriveTrainConstants {
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
        public static final double DRIVE_GEAR_REDUCTION = 6.0;

        public static final double STEERING_GEAR_REDUCTION = 3.0;

        public static final double DRIVE_CURRENT_LIMIT = 40.0;
        public static final double STEER_CURRENT_LIMIT = 20.0;

        public static final double NOMINAL_VOLTAGE = 12.0;

        // For talons PID full output is 1023 except for all FFF gains
        public static final PIDFFFGains DRIVE_VELOCITY_GAINS = new PIDFFFGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        public static final PIDGains STEER_POSITION_GAINS = new PIDGains(0.2, 0.0, 0.1);


        // Left right distance between center of wheels
        public static final double TRACKWIDTH_METERS = Units.inchesToMeters(24.78);

        // Front back distance between center of wheels
        public static final double WHEELBASE_METERS = Units.inchesToMeters(24.78);

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
                new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
                new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
                new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0)
        );

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
        public static final Constraints ANGULAR_CONSTRAINTS = new Constraints(
                MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        );

        public static final double ANGLE_TOLERANCE_DEGREES = 0.5;
        public static final double VELOCITY_TOLERANCE_METERS_PER_SECOND = 0.05;

        private static final SharedSwerveModuleConfiguration SHARED_SWERVE_MODULE_CONFIGURATION = new SharedSwerveModuleConfiguration(
                DRIVE_GEAR_REDUCTION, STEERING_GEAR_REDUCTION, DRIVE_CURRENT_LIMIT, STEER_CURRENT_LIMIT,
                NOMINAL_VOLTAGE, WHEEL_DIAMETER_METERS, MAX_VELOCITY_METERS_PER_SECOND, DRIVE_VELOCITY_GAINS,
                STEER_POSITION_GAINS
        );

        public static final SwerveModuleConfiguration FRONT_LEFT_MODULE_CONFIGURATION = new SwerveModuleConfiguration(
                1, 5, 9, true, true, 0.0, false, SHARED_SWERVE_MODULE_CONFIGURATION
        );

        public static final SwerveModuleConfiguration FRONT_RIGHT_MODULE_CONFIGURATION = new SwerveModuleConfiguration(
                2, 6, 10, true, true, 0.0, false, SHARED_SWERVE_MODULE_CONFIGURATION
        );

        public static final SwerveModuleConfiguration BACK_LEFT_MODULE_CONFIGURATION = new SwerveModuleConfiguration(
                3, 7, 11, true, true, 0.0, false, SHARED_SWERVE_MODULE_CONFIGURATION
        );

        public static final SwerveModuleConfiguration BACK_RIGHT_MODULE_CONFIGURATION = new SwerveModuleConfiguration(
                4, 8, 12, true, true, 0.0, false, SHARED_SWERVE_MODULE_CONFIGURATION
        );
    }
}
