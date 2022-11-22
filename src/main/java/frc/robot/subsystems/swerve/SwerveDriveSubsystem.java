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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.util.InstantRunWhenDisabledCommand;
import frc.robot.telemetry.types.DoubleArrayTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.StringTelemetryEntry;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.SwerveUtils;


import static frc.robot.Constants.DriveTrainConstants.*;

/**
 * The subsystem containing all the swerve modules
 */
public class SwerveDriveSubsystem extends SubsystemBase {
    enum DriveMode {
        OPEN_LOOP,
        CLOSE_LOOP,
        CHARACTERIZATION
    }

    private final SwerveModule[] modules = new SwerveModule[NUM_MODULES];

    private final AHRS gyro = new AHRS();
//    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
//            getGyroRotation(), new Pose2d(), KINEMATICS, STATE_STD_DEVS, LOCAL_MEASUREMENT_STD_DEVS, VISION_STD_DEVS
//    );
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(KINEMATICS, getGyroRotation());

    private final Alert navXNotConnectedFaultAlert = new Alert(
            "navX is not connected. Field-centric drive and odometry will be negatively effected!", AlertType.ERROR
    );
    private final Alert navXCalibratingAlert = new Alert("navX is calibrating. Keep the robot still!", AlertType.INFO);
    private final DoubleTelemetryEntry gyroEntry = new DoubleTelemetryEntry("/drive/gyroDegrees", true);
    private final DoubleArrayTelemetryEntry odometryEntry = new DoubleArrayTelemetryEntry(
            "/drive/estimatedPose", false
    );
    private final StringTelemetryEntry driveEventLogger = new StringTelemetryEntry("/drive/events", false, false);

    private final Field2d field2d = new Field2d();

    private SwerveModuleState[] desiredStates = new SwerveModuleState[4];
    private SwerveModuleState[] nextStates = new SwerveModuleState[4];
    private boolean activeSteer = true;
    private DriveMode driveMode = DriveMode.OPEN_LOOP;
    private double characterizationVoltage = 0.0;

    public SwerveDriveSubsystem() {
        modules[0] = new SwerveModule(FRONT_LEFT_MODULE_CONFIGURATION);
        modules[1] = new SwerveModule(FRONT_RIGHT_MODULE_CONFIGURATION);
        modules[2] = new SwerveModule(BACK_LEFT_MODULE_CONFIGURATION);
        modules[3] = new SwerveModule(BACK_RIGHT_MODULE_CONFIGURATION);
        driveEventLogger.append("Swerve modules initialized");

        ShuffleboardTab driveTab = Shuffleboard.getTab("DriveTrainRaw");

        driveTab.add("Field", field2d);
        driveTab.add(
                "Reset to Absolute", new InstantRunWhenDisabledCommand(this::setAllModulesToAbsolute).withName("Reset")
        );
        driveTab.addBoolean("All have been set to absolute", this::allModulesAtAbsolute);

        stopMovement();
    }

    /**
     * @return the value from the gyro. This does not get reset when resetOdometry
     *         is called. Use <code>getPose().getRotation2d()</code> to get the
     *         field centric value. Counterclockwise is positive.
     */
    private Rotation2d getGyroRotation() {
        // We prefer to use this as it hypothetically has zero drift
        if (gyro.isMagnetometerCalibrated()) {
            return Rotation2d.fromDegrees(gyro.getFusedHeading());
        }
        // It is mounted upside-down so no invert
        return Rotation2d.fromDegrees(gyro.getYaw());
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

    public void resetOdometry() {
        resetOdometry(new Pose2d());
    }

    /**
     * Set the current perceived location of the robot to the provided pose
     *
     * @param pose2d the provided pose
     */
    public void resetOdometry(Pose2d pose2d) {
//        poseEstimator.resetPosition(pose2d, getGyroRotation());
        odometry.resetPosition(pose2d, getGyroRotation());

        driveEventLogger.append("Odometry reset");
    }

    /**
     * @return the estimated position of the robot
     */
    public Pose2d getPose() {
//        return poseEstimator.getEstimatedPosition();
        return odometry.getPoseMeters();
    }

    public ChassisSpeeds getCurrentChassisSpeeds() {
        return KINEMATICS.toChassisSpeeds(getActualStates());
    }

    /**
     * Set the desired speed of the robot. Chassis speeds are always robot centric
     * but can be created from field centric values through
     * {@link ChassisSpeeds#fromFieldRelativeSpeeds(double, double, double, Rotation2d)}
     *
     * @param chassisSpeeds the desired chassis speeds
     * @param openLoop      if true then velocity will be handled exclusivity with
     *                      feedforward (mostly used for teleop). If false a PIDF
     *                      will be used (mostly used for auto)
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean openLoop) {
        setRawStates(true, openLoop, KINEMATICS.toSwerveModuleStates(chassisSpeeds));
    }

    /**
     * Set the desired speed of the robot. Chassis speeds are always robot centric
     * but can be created from field centric values through
     * {@link ChassisSpeeds#fromFieldRelativeSpeeds(double, double, double, Rotation2d)}
     *
     * @param chassisSpeeds     the desired chassis speeds
     * @param nextChassisSpeeds the speeds that will be next, used for calculating
     *                          acceleration
     * @param openLoop          if true then velocity will be handled exclusivity
     *                          with feedforward (mostly used for teleop). If false
     *                          a PIDF will be used (mostly used for auto)
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, ChassisSpeeds nextChassisSpeeds, boolean openLoop) {
        setRawStates(
                true, openLoop, KINEMATICS.toSwerveModuleStates(chassisSpeeds),
                KINEMATICS.toSwerveModuleStates(nextChassisSpeeds)
        );
    }

    /**
     * Sets the desired swerve drive states for the modules. This method also takes
     * a copy of the states, so they will not be changed. Assumes zero acceleration.
     *
     * @param activeSteer if false will not actively power the steering motor
     * @param openLoop    if true then velocity will be handled exclusivity with
     *                    feedforward (for teleop mostly). If false a PIDF will be
     *                    used (for auto)
     * @param states      the desired states... Ordered front left, front right,
     *                    back left, back right
     */
    public void setRawStates(boolean activeSteer, boolean openLoop, SwerveModuleState[] states) {
        setRawStates(activeSteer, openLoop, states, states);
    }

    /**
     * Sets the desired swerve drive states for the modules. This method also takes
     * a copy of the states, so they will not be changed
     *
     * @param activeSteer if false will not actively power the steering motor
     * @param openLoop    if true then velocity will be handled exclusivity with
     *                    feedforward (for teleop mostly). If false a PIDF will be
     *                    used (for auto)
     * @param states      the desired states... Ordered front left, front right,
     *                    back left, back right
     * @param nextStates  the states that will be used for the acceleration ff
     */
    public void setRawStates(
            boolean activeSteer, boolean openLoop, SwerveModuleState[] states, SwerveModuleState[] nextStates
    ) {
        if (states.length != modules.length) {
            throw new IllegalArgumentException("You must provide states for all modules");
        }

        driveMode = openLoop ? DriveMode.OPEN_LOOP : DriveMode.CLOSE_LOOP;
        this.activeSteer = activeSteer;

        this.desiredStates = SwerveUtils.copySwerveStateArray(states);
        this.nextStates = SwerveUtils.copySwerveStateArray(nextStates);
    }

    /**
     * Sets each module velocity to zero and desired angle to what it currently is
     */
    public void stopMovement() {
        SwerveModuleState[] newStates = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            newStates[i] = new SwerveModuleState(0.0, modules[i].getActualState().angle);
        }
        setRawStates(false, true, newStates);
    }

    public void setCharacterizationVoltage(double voltage) {
        driveMode = DriveMode.CHARACTERIZATION;
        characterizationVoltage = voltage;
    }

    public double getAverageDriveVelocityMetersSecond() {
        SwerveModuleState[] actualStates = getActualStates();
        double sum = 0.0;
        for (SwerveModuleState state : actualStates) {
            sum += state.speedMetersPerSecond;
        }

        return sum / actualStates.length;
    }

    /**
     * @return true if all modules are at the set desired states within the
     *         threshold in {@link frc.robot.Constants}
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
                actualState.angle.getDegrees(), desiredState.angle.getDegrees(), ANGLE_TOLERANCE_RADIANS
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

    private SwerveModuleState[] getActualStates() {
        SwerveModuleState[] actualStates = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            actualStates[i] = modules[i].getActualState();
        }
        return actualStates;
    }

    private static boolean inTolerance(double val, double target, double tolerance) {
        return Math.abs(target - val) <= tolerance;
    }

    @Override
    public void periodic() {
        switch (driveMode) {
            case OPEN_LOOP:
            case CLOSE_LOOP:
                SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_PER_SECOND);

                for (int i = 0; i < modules.length; i++) {
                    modules[i].setDesiredState(
                            desiredStates[i], nextStates[i], activeSteer, driveMode == DriveMode.OPEN_LOOP
                    );
                }
                break;
            case CHARACTERIZATION:
                for (SwerveModule module : modules) {
                    module.setCharacterizationVoltage(characterizationVoltage);
                }
                break;
        }

//        poseEstimator.update(getGyroRotation(), actualStates);
        odometry.update(getGyroRotation(), getActualStates());

        logValues();
    }

    private void logValues() {
        gyroEntry.append(getGyroRotation().getRadians());

        Pose2d estimatedPose = getPose();
        odometryEntry.append(
                new double[] { estimatedPose.getX(), estimatedPose.getY(), estimatedPose.getRotation().getRadians() }
        );

        field2d.setRobotPose(estimatedPose);
        for (int i = 0; i < modules.length; i++) {
            field2d.getObject("module " + i).setPose(
                    new Pose2d(MODULE_TRANSLATIONS[i], modules[i].getActualState().angle).relativeTo(estimatedPose)
            );
        }

        for (SwerveModule module : modules) {
            module.logValues();
        }

        navXNotConnectedFaultAlert.set(!gyro.isConnected());
        navXCalibratingAlert.set(gyro.isCalibrating());
    }
}
