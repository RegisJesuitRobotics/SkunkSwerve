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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveTrainConstants.*;


public class SwerveDriveSubsystem extends SubsystemBase {
    private final SwerveModule[] modules = new SwerveModule[4];

    private final AHRS gyro = new AHRS();

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(KINEMATICS, getGyroRotation());

    private SwerveModuleState[] desiredStates = new SwerveModuleState[4];

    public SwerveDriveSubsystem() {
        modules[0] = new SwerveModule(FRONT_LEFT_MODULE_CONFIGURATION);
        modules[1] = new SwerveModule(FRONT_RIGHT_MODULE_CONFIGURATION);
        modules[2] = new SwerveModule(BACK_LEFT_MODULE_CONFIGURATION);
        modules[3] = new SwerveModule(BACK_RIGHT_MODULE_CONFIGURATION);

        ShuffleboardTab driveTab = Shuffleboard.getTab("DriveTrainRaw");

        driveTab.add("Front Left", modules[0]).withSize(2, 3);
        driveTab.add("Front Right", modules[1]).withSize(2, 3);
        driveTab.add("Back Left", modules[2]).withSize(2, 3);
        driveTab.add("Back Right", modules[3]).withSize(2, 3);

        stopMovement();
    }

    public void zeroGyro() {
        gyro.zeroYaw();
    }

    public Rotation2d getGyroRotation() {
        // We prefer to use this as it hypothetically has zero drift
        if (gyro.isMagnetometerCalibrated()) {
            // TODO: should this be negative
            return Rotation2d.fromDegrees(-gyro.getFusedHeading());
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
        desiredStates = KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    }

    /**
     * @param states ordered front left, front right, back left, back right
     */
    public void setRawStates(SwerveModuleState... states) {
        if (states.length != modules.length) {
            throw new IllegalArgumentException("You must provide states for all modules");
        }

        this.desiredStates = states;
    }

    public void stopMovement() {
        for (int i = 0; i < desiredStates.length; i++) {
            desiredStates[i] = new SwerveModuleState(0.0, modules[i].getActualState().angle);
        }
    }

    public boolean atDesiredStates() {
        boolean atStates = true;
        for (int i = 0; i < modules.length; i++) {
            atStates &= moduleAtDesiredState(i);
        }
        return atStates;
    }

    private boolean moduleAtDesiredState(int index) {
        SwerveModule module = modules[index];
        SwerveModuleState desiredState = desiredStates[index];
        SwerveModuleState actualState = module.getActualState();

        boolean atState = inTolerance(actualState.speedMetersPerSecond, desiredState.speedMetersPerSecond,
                VELOCITY_TOLERANCE_METERS_PER_SECOND);
        atState &= inTolerance(actualState.angle.getDegrees(), desiredState.angle.getDegrees(),
                ANGLE_TOLERANCE_RADIANS);
        return atState;
    }

    private static boolean inTolerance(double val, double target, double tolerance) {
        return Math.abs(target - val) <= tolerance;
    }

    @Override
    public void periodic() {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_PER_SECOND);

        SwerveModuleState[] actualStates = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            SwerveModule module = modules[i];

            module.setDesiredState(desiredStates[i]);
            actualStates[i] = module.getActualState();
        }

        odometry.update(getGyroRotation(), actualStates);
    }
}
