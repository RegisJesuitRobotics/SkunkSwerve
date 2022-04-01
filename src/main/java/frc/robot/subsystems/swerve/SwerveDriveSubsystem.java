// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DriveTrainConstants.*;


public class SwerveDriveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final AHRS gyro = new AHRS();

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(KINEMATICS, getGyroRotation());

    private SwerveModuleState[] states = new SwerveModuleState[4];

    public SwerveDriveSubsystem() {
        this.frontLeft = new SwerveModule(FRONT_LEFT_MODULE_CONFIGURATION);
        this.frontRight = new SwerveModule(FRONT_RIGHT_MODULE_CONFIGURATION);
        this.backLeft = new SwerveModule(BACK_LEFT_MODULE_CONFIGURATION);
        this.backRight = new SwerveModule(BACK_RIGHT_MODULE_CONFIGURATION);
    }

    public void zeroGyro() {
        gyro.zeroYaw();
    }

    public Rotation2d getGyroRotation() {
        // We prefer to use this as it hypothetically has zero drift
        if (gyro.isMagnetometerCalibrated()) {
            return Rotation2d.fromDegrees(gyro.getFusedHeading());
        }

        return Rotation2d.fromDegrees(-gyro.getYaw());
    }

    public void resetOdometry(Pose2d pose2d) {
        odometry.resetPosition(pose2d, getGyroRotation());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        states = KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    }

    /**
     * @param states ordered front left, front right, back left, back right
     */
    public void setRawStates(SwerveModuleState[] states) {
        if (states.length != 4) {
            throw new IllegalArgumentException("You must provide all 4 states");
        }
        this.states = states;
    }

    public void stopMovement() {
        for (SwerveModuleState state : states) {
            state.speedMetersPerSecond = 0.0;
        }
    }

    public boolean atDesiredStates() {
        boolean atStates = moduleAtDesiredState(frontLeft, 0);
        atStates &= moduleAtDesiredState(backLeft, 2);
        atStates &= moduleAtDesiredState(frontRight, 1);
        atStates &= moduleAtDesiredState(backRight, 3);
        return atStates;
    }

    private boolean moduleAtDesiredState(SwerveModule module, int index) {
        boolean atState = inTolerance(module.getDriveMotorVelocityMetersPerSecond(), states[index].speedMetersPerSecond,
                VELOCITY_TOLERANCE_METERS_PER_SECOND);
        atState &= inTolerance(module.getSteeringAngleDegrees(), states[index].angle.getDegrees(), ANGLE_TOLERANCE_RADIANS);
        return atState;
    }

    private static boolean inTolerance(double val, double target, double tolerance) {
        return Math.abs(target - val) <= tolerance;
    }

    @Override
    public void periodic() {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);

        odometry.update(getGyroRotation(), states);
    }

}
