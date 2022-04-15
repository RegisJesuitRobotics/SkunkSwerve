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
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DriveTrainConstants.*;


public class SwerveDriveSubsystem extends SubsystemBase {
    private final SwerveModule[] modules = new SwerveModule[4];
    private final AHRS gyro = new AHRS();
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(KINEMATICS, getGyroRotation());

    private final DataLog logger = DataLogManager.getLog();
    private final DoubleLogEntry gyroEntry = new DoubleLogEntry(logger, "/drive/gyroDegrees");
    private final DoubleLogEntry odometryXEntry = new DoubleLogEntry(logger, "/drive/estimatedX");
    private final DoubleLogEntry odometryYEntry = new DoubleLogEntry(logger, "/drive/estimatedY");
    private final DoubleLogEntry odometryHeadingEntry = new DoubleLogEntry(logger, "/drive/estimatedHeading");
    private final StringLogEntry driveEventLogger = new StringLogEntry(logger, "/drive/events");

    private SwerveModuleState[] desiredStates = new SwerveModuleState[4];

    public SwerveDriveSubsystem() {
        modules[0] = new SwerveModule(FRONT_LEFT_MODULE_CONFIGURATION);
        modules[1] = new SwerveModule(FRONT_RIGHT_MODULE_CONFIGURATION);
        modules[2] = new SwerveModule(BACK_LEFT_MODULE_CONFIGURATION);
        modules[3] = new SwerveModule(BACK_RIGHT_MODULE_CONFIGURATION);
        driveEventLogger.append("Swerve modules initialized");

        ShuffleboardTab driveTab = Shuffleboard.getTab("DriveTrainRaw");

        driveTab.add("Front Left", modules[0]).withSize(2, 3);
        driveTab.add("Front Right", modules[1]).withSize(2, 3);
        driveTab.add("Back Left", modules[2]).withSize(2, 3);
        driveTab.add("Back Right", modules[3]).withSize(2, 3);

        stopMovement();
    }

    public void zeroGyro() {
        gyro.zeroYaw();
        driveEventLogger.append("Gyro reset");
    }

    /**
     * @return the value from the gyro. This does not get reset when resetOdometry
     *         is called
     */
    public Rotation2d getGyroRotation() {
        // We prefer to use this as it hypothetically has zero drift
        if (gyro.isMagnetometerCalibrated()) {
            // TODO: should this be negative?
            return Rotation2d.fromDegrees(-gyro.getFusedHeading());
        }

        return Rotation2d.fromDegrees(-gyro.getYaw());
    }

    public void resetOdometry(Pose2d pose2d) {
        odometry.resetPosition(pose2d, getGyroRotation());

        driveEventLogger.append("Odometry reset");
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean openLoop) {
        setRawStates(openLoop, KINEMATICS.toSwerveModuleStates(chassisSpeeds));
    }

    /**
     * @param states ordered front left, front right, back left, back right
     */
    public void setRawStates(boolean openLoop, SwerveModuleState... states) {
        if (states.length != modules.length) {
            throw new IllegalArgumentException("You must provide states for all modules");
        }

        this.desiredStates = states;
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(desiredStates[i], openLoop);
        }
    }

    public void stopMovement() {
        setChassisSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0), true);
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
                ANGLE_TOLERANCE_DEGREES);
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
            actualStates[i] = modules[i].getActualState();
        }

        odometry.update(getGyroRotation(), actualStates);

        logValues();
    }

    private void logValues() {
        gyroEntry.append(getGyroRotation().getDegrees());

        Pose2d estimatedPose = getPose();
        odometryXEntry.append(estimatedPose.getX());
        odometryYEntry.append(estimatedPose.getY());
        odometryHeadingEntry.append(estimatedPose.getRotation().getDegrees());

        for (SwerveModule module : modules) {
            module.logValues();
        }
    }
}
