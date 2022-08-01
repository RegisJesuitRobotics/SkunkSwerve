// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.util.InstantRunWhenDisabledCommand;
import frc.robot.utils.SwerveUtils;

import static frc.robot.Constants.DriveTrainConstants.*;

/**
 * The subsystem containing all the swerve modules
 */
public class SwerveDriveSubsystem extends SubsystemBase {
    private final SwerveModule[] modules = new SwerveModule[4];
    private final AHRS gyro = new AHRS();
    private final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(
            getGyroRotation(), new Pose2d(), KINEMATICS, STATE_STD_DEVS, LOCAL_MEASUREMENT_STD_DEVS, VISION_STD_DEVS
    );

    private final DataLog logger = DataLogManager.getLog();
    private final DoubleLogEntry gyroEntry = new DoubleLogEntry(logger, "/drive/gyroDegrees");
    private final DoubleLogEntry odometryXEntry = new DoubleLogEntry(logger, "/drive/estimatedX");
    private final DoubleLogEntry odometryYEntry = new DoubleLogEntry(logger, "/drive/estimatedY");
    private final DoubleLogEntry odometryHeadingEntry = new DoubleLogEntry(logger, "/drive/estimatedHeading");
    private final StringLogEntry driveEventLogger = new StringLogEntry(logger, "/drive/events");

    private final Field2d field2d = new Field2d();

    private SwerveModuleState[] desiredStates = new SwerveModuleState[4];
    private boolean openLoop = true;

    public SwerveDriveSubsystem() {
        modules[0] = new SwerveModule(FRONT_LEFT_MODULE_CONFIGURATION);
        modules[1] = new SwerveModule(FRONT_RIGHT_MODULE_CONFIGURATION);
        modules[2] = new SwerveModule(BACK_LEFT_MODULE_CONFIGURATION);
        modules[3] = new SwerveModule(BACK_RIGHT_MODULE_CONFIGURATION);
        driveEventLogger.append("Swerve modules initialized");

        ShuffleboardTab driveTab = Shuffleboard.getTab("DriveTrainRaw");

        driveTab.add("Field", field2d);
        driveTab.add("Front Left", modules[0]).withSize(2, 3);
        driveTab.add("Front Right", modules[1]).withSize(2, 3);
        driveTab.add("Back Left", modules[2]).withSize(2, 3);
        driveTab.add("Back Right", modules[3]).withSize(2, 3);

        driveTab.add("Reset to Absolute", new InstantRunWhenDisabledCommand(this::setAllModulesToAbsolute));
        driveTab.addBoolean("All have been set to absolute", this::allModulesAtAbsolute);

        stopMovement();
    }

    /**
     * @return the value from the gyro. This does not get reset when resetOdometry
     *         is called. Use <code>getPose().getRotation2d()</code> to get the
     *         field centric value. Counterclockwise is positive (unit circle like).
     */
    private Rotation2d getGyroRotation() {
        // We prefer to use this as it hypothetically has zero drift
        if (gyro.isMagnetometerCalibrated()) {
            return Rotation2d.fromDegrees(gyro.getFusedHeading());
        }
        return Rotation2d.fromDegrees(-gyro.getYaw());
    }

    /**
     * Sets the odometry perceived location to zero
     */
    public void zeroHeading() {
        setHeading(Rotation2d.fromDegrees(0.0));
    }

    /**
     * Set the odometry perceived location to the provided heading
     *
     * @param newHeading the provided heading
     */
    public void setHeading(Rotation2d newHeading) {
        Pose2d currentPose = getPose();

        Pose2d newPose = new Pose2d(currentPose.getTranslation(), newHeading);

        resetOdometry(newPose);
    }

    /**
     * Set the current perceived location of the robot to the provided pose
     *
     * @param pose2d the provided pose
     */
    public void resetOdometry(Pose2d pose2d) {
        odometry.resetPosition(pose2d, getGyroRotation());

        driveEventLogger.append("Odometry reset");
    }

    /**
     * @return the estimated position of the robot
     */
    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    /**
     * Set the desired speed of the robot. Chassis speeds are always robot centric
     * but can be created from field centric values through
     * <code>ChassisSpeeds.fromFieldRelativeSpeeds</code>
     *
     * @param chassisSpeeds the desired chassis speeds
     * @param openLoop      if true then velocity will be handled exclusivity with
     *                      feedforward (mostly used for teleop). If false a PIDF
     *                      will be used (mostly used for auto)
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean openLoop) {
        setRawStates(openLoop, KINEMATICS.toSwerveModuleStates(chassisSpeeds));
    }

    /**
     * Sets the desired swerve drive states for the modules. This method also takes
     * a copy of the states, so they will not be changed
     *
     * @param openLoop if true then velocity will be handled exclusivity with
     *                 feedforward (for teleop mostly). If false a PIDF will be used
     *                 (for auto)
     * @param states   the desired states... Ordered front left, front right, back
     *                 left, back right
     */
    public void setRawStates(boolean openLoop, SwerveModuleState... states) {
        if (states.length != modules.length) {
            throw new IllegalArgumentException("You must provide states for all modules");
        }

        this.openLoop = openLoop;

        // Deep copy of states array
        desiredStates = new SwerveModuleState[states.length];
        for (int i = 0; i < states.length; i++) {
            desiredStates[i] = SwerveUtils.copySwerveState(states[i]);
        }
    }

    /**
     * Sets each module velocity to zero and desired angle to what it currently is
     */
    public void stopMovement() {
        openLoop = true;
        for (int i = 0; i < modules.length; i++) {
            SwerveModule module = modules[i];
            // Copy the current angle so everything stops moving
            desiredStates[i] = new SwerveModuleState(0.0, module.getActualState().angle);
        }
    }

    /**
     * @return true if all modules are at the set desired states within the
     *         threshold in <code>Constants.java</code>
     */
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

        boolean atState = inTolerance(
                actualState.speedMetersPerSecond, desiredState.speedMetersPerSecond,
                VELOCITY_TOLERANCE_METERS_PER_SECOND
        );
        atState &= inTolerance(
                actualState.angle.getDegrees(), desiredState.angle.getDegrees(), ANGLE_TOLERANCE_DEGREES
        );
        return atState;
    }

    public void setAllModulesToAbsolute() {
        for (SwerveModule module : modules) {
            module.resetSteeringToAbsolute();
        }
    }

    private boolean allModulesAtAbsolute() {
        boolean allSet = true;
        for (SwerveModule module : modules) {
            allSet &= module.isSetToAbsolute();
        }
        return allSet;
    }

    private static boolean inTolerance(double val, double target, double tolerance) {
        return Math.abs(target - val) <= tolerance;
    }

    @Override
    public void periodic() {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_PER_SECOND);

        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(desiredStates[i], openLoop);
        }

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

        field2d.setRobotPose(estimatedPose);

        for (SwerveModule module : modules) {
            module.logValues();
        }
    }
}
