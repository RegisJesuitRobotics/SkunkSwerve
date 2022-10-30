package frc.robot.subsystems.swerve;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.util.InstantRunWhenDisabledCommand;
import frc.robot.logging.LoggableTalonFX;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.PIDFFFGains;
import frc.robot.utils.PIDGains;
import frc.robot.utils.SwerveUtils;

public class SwerveModule implements Sendable {
    private static final int CAN_TIMEOUT_MS = 250;
    private static final double CANCODER_INITIAL_TIMEOUT_SECONDS = 5.0;
    private static final int STATUS_1_PERIOD_MS = 60;
    private static final int CANCODER_PERIOD_MS = 55;

    private static int instances = 0;

    private final int instanceId;
    private final DoubleLogEntry desiredVelocityEntry;
    private final BooleanLogEntry openLoopEntry;
    private final DoubleLogEntry desiredHeadingEntry;
    private final DoubleLogEntry actualVelocityEntry;
    private final DoubleLogEntry actualHeadingEntry;
    private final DoubleLogEntry absoluteHeadingEntry;
    private final BooleanLogEntry setToAbsoluteEntry;
    private final StringLogEntry moduleEventEntry;

    private final Alert notSetToAbsoluteAlert;
    private final Alert steeringEncoderFaultAlert;
    private final Alert steeringMotorFaultAlert;
    private final Alert driveMotorFaultAlert;
    private final Alert inDeadModeAlert;

    private final LoggableTalonFX driveMotor;
    private final LoggableTalonFX steeringMotor;
    private final CANCoder absoluteSteeringEncoder;

    private final double driveMotorConversionFactorVelocity;
    private final double steeringMotorConversionFactorPosition;
    private final double nominalVoltage;
    private final double steeringEncoderOffset;

    private final SimpleMotorFeedforward driveMotorFF;
    private final double openLoopMaxSpeed;

    private SwerveModuleState desiredState;
    private boolean setToAbsolute = false;
    private boolean isDeadMode = false;
    private double lastMoveTime = 0.0;
    private double lastAbsoluteResetTime = 0.0;

    /**
     * Constructs a new Swerve Module using the given config
     */
    public SwerveModule(SwerveModuleConfiguration config) {
        instanceId = instances++;

        // Initialize all logging entries
        DataLog logger = DataLogManager.getLog();
        String tableName = "/drive/modules/" + instanceId + "/";
        desiredVelocityEntry = new DoubleLogEntry(logger, tableName + "desiredVelocity");
        openLoopEntry = new BooleanLogEntry(logger, tableName + "openLoop");
        desiredHeadingEntry = new DoubleLogEntry(logger, tableName + "desiredHeading");
        actualVelocityEntry = new DoubleLogEntry(logger, tableName + "actualVelocity");
        actualHeadingEntry = new DoubleLogEntry(logger, tableName + "actualHeading");
        absoluteHeadingEntry = new DoubleLogEntry(logger, tableName + "absoluteHeading");
        setToAbsoluteEntry = new BooleanLogEntry(logger, tableName + "setToAbsolute");
        moduleEventEntry = new StringLogEntry(logger, tableName + "events");

        String alertBeginning = "Module " + instanceId + ": ";
        notSetToAbsoluteAlert = new Alert(
                alertBeginning + "Steering is not reset to absolute position", AlertType.ERROR
        );
        steeringEncoderFaultAlert = new Alert(
                alertBeginning + "Steering encoder had a fault initializing (see logs)", AlertType.ERROR
        );
        steeringMotorFaultAlert = new Alert(
                alertBeginning + "Steering motor had a fault initializing (see logs)", AlertType.ERROR
        );
        driveMotorFaultAlert = new Alert(
                alertBeginning + "Drive motor had a fault initializing (see logs)", AlertType.ERROR
        );
        inDeadModeAlert = new Alert(alertBeginning + "In dead mode", AlertType.WARNING);

        this.driveMotorConversionFactorVelocity = (config.sharedConfiguration.wheelDiameterMeters * Math.PI * 10)
                / (config.sharedConfiguration.driveGearRatio * 2048);
        this.steeringMotorConversionFactorPosition = (Math.PI * 2) / (config.sharedConfiguration.steerGearRatio * 2048);

        // Drive motor
        this.driveMotor = new LoggableTalonFX(config.driveMotorPort, tableName + "driveMotor");
        configDriveMotor(config);

        // Steer encoder
        this.absoluteSteeringEncoder = new WPI_CANCoder(config.steeringEncoderPort);
        configSteeringEncoder(config);

        // Steer motor
        this.steeringMotor = new LoggableTalonFX(config.steeringMotorPort, tableName + "steeringMotor");
        configSteeringMotor(config);

        this.nominalVoltage = config.sharedConfiguration.nominalVoltage;
        this.steeringEncoderOffset = config.offsetDegrees;

        this.driveMotorFF = config.sharedConfiguration.driveVelocityGains.feedforward;
        this.openLoopMaxSpeed = config.sharedConfiguration.openLoopMaxSpeed;

        resetSteeringToAbsolute(CANCODER_INITIAL_TIMEOUT_SECONDS);
    }

    private void configDriveMotor(SwerveModuleConfiguration config) {
        boolean faultInitializing = checkCTREError(
                driveMotor.configFactoryDefault(CAN_TIMEOUT_MS), "Could not config drive motor factory default"
        );

        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

        // Current limit
        motorConfiguration.supplyCurrLimit.currentLimit = config.sharedConfiguration.driveContinuousCurrentLimit;
        motorConfiguration.supplyCurrLimit.triggerThresholdCurrent = config.sharedConfiguration.drivePeakCurrentLimit;
        motorConfiguration.supplyCurrLimit.triggerThresholdTime = config.sharedConfiguration.drivePeakCurrentDurationSeconds;
        motorConfiguration.supplyCurrLimit.enable = true;
        // Voltage compensation
        motorConfiguration.voltageCompSaturation = config.sharedConfiguration.nominalVoltage;
        config.sharedConfiguration.driveVelocityGains.setSlot(motorConfiguration.slot0);

        faultInitializing |= checkCTREError(
                driveMotor.configAllSettings(motorConfiguration, CAN_TIMEOUT_MS), "Could not configure drive motor"
        );

        driveMotor.setInverted(
                config.driveMotorInverted ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise
        );

        faultInitializing |= checkCTREError(
                driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MS),
                "Could not config drive motor sensor"
        );
        driveMotor.setSensorPhase(true);

        driveMotor.setNeutralMode(NeutralMode.Brake);

        // We don't really need the information on this status frame, so we can make it
        // not send as often to save CAN bandwidth
        faultInitializing |= checkCTREError(
                driveMotor.setStatusFramePeriod(StatusFrame.Status_1_General, STATUS_1_PERIOD_MS, CAN_TIMEOUT_MS),
                "Could not config drive status frame"
        );

        driveMotorFaultAlert.set(faultInitializing);

        // Clear the reset of us starting up
        driveMotor.hasResetOccurred();

        moduleEventEntry.append("Drive motor initialized" + (faultInitializing ? " with faults" : ""));
    }

    private void configSteeringMotor(SwerveModuleConfiguration config) {
        boolean faultInitializing = checkCTREError(
                steeringMotor.configFactoryDefault(), "Could not config steer motor factory default"
        );

        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

        // Current limit
        motorConfiguration.supplyCurrLimit.currentLimit = config.sharedConfiguration.steerContinuousCurrentLimit;
        motorConfiguration.supplyCurrLimit.triggerThresholdCurrent = config.sharedConfiguration.steerPeakCurrentLimit;
        motorConfiguration.supplyCurrLimit.triggerThresholdTime = config.sharedConfiguration.steerPeakCurrentDurationSeconds;
        motorConfiguration.supplyCurrLimit.enable = true;
        // Voltage compensation
        motorConfiguration.voltageCompSaturation = config.sharedConfiguration.nominalVoltage;
        config.sharedConfiguration.steerPositionGains.setSlot(motorConfiguration.slot0);
        motorConfiguration.slot0.allowableClosedloopError = config.sharedConfiguration.angleErrorToleranceRadians
                / steeringMotorConversionFactorPosition;

        faultInitializing |= checkCTREError(
                steeringMotor.configAllSettings(motorConfiguration, CAN_TIMEOUT_MS), "Could not configure steer motor"
        );
        steeringMotor.setInverted(
                config.steeringMotorInverted ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise
        );

        faultInitializing |= checkCTREError(
                steeringMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MS),
                "Could not config steer motor sensor"
        );

        // Because + on motor is clockwise, and we want + on encoder to be
        // counter-clockwise we have to set the sensor phase
        steeringMotor.setSensorPhase(true);

        steeringMotor.setNeutralMode(NeutralMode.Brake);

        // We don't really need the information on this status frame, so we can make it
        // not send as often to save CAN bandwidth
        faultInitializing |= checkCTREError(
                steeringMotor.setStatusFramePeriod(StatusFrame.Status_1_General, STATUS_1_PERIOD_MS, CAN_TIMEOUT_MS),
                "Could not config steer status frame"
        );

        steeringMotorFaultAlert.set(faultInitializing);

        // Clear the reset of us starting up
        steeringMotor.hasResetOccurred();

        moduleEventEntry.append("Steer motor initialized" + (faultInitializing ? " with faults" : ""));
    }

    private void configSteeringEncoder(SwerveModuleConfiguration config) {
        CANCoderConfiguration encoderConfiguration = new CANCoderConfiguration();

        encoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        encoderConfiguration.sensorDirection = config.steeringEncoderInverted;

        boolean faultInitializing = checkCTREError(
                absoluteSteeringEncoder.configAllSettings(encoderConfiguration, CAN_TIMEOUT_MS),
                "Could not configure steer encoder"
        );

        // Because we are only reading this at the beginning we do not have to update it
        // often
        faultInitializing |= checkCTREError(
                absoluteSteeringEncoder
                        .setStatusFramePeriod(CANCoderStatusFrame.SensorData, CANCODER_PERIOD_MS, CAN_TIMEOUT_MS),
                "Could not set steer encoder status frame"
        );

        steeringEncoderFaultAlert.set(faultInitializing);
        moduleEventEntry.append("Steer encoder initialized" + (faultInitializing ? " with faults" : ""));
    }

    /**
     * Reports error to event log and driver station. Prepends module details to DS
     * report.
     *
     * @param message The message to report
     */
    private void reportError(String message) {
        moduleEventEntry.append(message);
        DriverStation.reportError(String.format("Module %d: %s", instanceId, message), false);
    }

    /**
     *
     * @param errorCode the error code passed by CTRE
     * @param message   the message explaining the error
     * @return true of there was an error, false if there wasn't
     */
    private boolean checkCTREError(ErrorCode errorCode, String message) {
        if (errorCode != ErrorCode.OK) {
            String fullMessage = String.format("%s: %s", message, errorCode.toString());

            reportError(fullMessage);
            return true;
        }
        return false;
    }

    private void checkForSteeringMotorReset() {
        // Steering motor lost power
        if (steeringMotor.hasResetOccurred()) {
            reportError("Steering motor reset occurred");
            setToAbsolute = false;
            resetSteeringToAbsolute();
        }
    }

    /**
     * Resets the integrated encoder on the steering motor to the absolute position
     * of the CANCoder. Trys only once, and if it fails, it will not try again until
     */
    public void resetSteeringToAbsolute() {
        resetSteeringToAbsolute(0.0);
    }

    /**
     * Resets the integrated encoder on the steering motor to the absolute position
     * of the CANCoder
     *
     * @param timeout The timeout in seconds to wait.
     */
    public void resetSteeringToAbsolute(double timeout) {
        double startTime = Timer.getFPGATimestamp();
        setToAbsolute = false;

        double absolutePosition;
        boolean gotAbsolutePosition = false;
        do {
            absolutePosition = getAbsoluteDegrees();
            // If no error
            if (!checkCTREError(
                    absoluteSteeringEncoder.getLastError(), "Could not get absolute position from encoder"
            )) {
                gotAbsolutePosition = true;
                break;
            }
        } while (timeout != 0.0 || Timer.getFPGATimestamp() - startTime < timeout);

        if (gotAbsolutePosition) {
            ErrorCode settingPositionError = steeringMotor.setSelectedSensorPosition(
                    Units.degreesToRadians(absolutePosition) / steeringMotorConversionFactorPosition, 0, CAN_TIMEOUT_MS
            );
            // If no error
            if (!checkCTREError(settingPositionError, "Could not set steer motor encoder to absolute position")) {
                setToAbsolute = true;
                lastAbsoluteResetTime = Timer.getFPGATimestamp();
                moduleEventEntry.append("Reset steer motor encoder to position: " + absolutePosition);
            }
        } else {
            reportError("CANCoder timed out while trying to get absolute position");
        }
        notSetToAbsoluteAlert.set(!setToAbsolute);
    }

    public boolean isSetToAbsolute() {
        return setToAbsolute;
    }

    private double getAbsoluteDegrees() {
        return Math.IEEEremainder(absoluteSteeringEncoder.getAbsolutePosition() + steeringEncoderOffset, 360);
    }

    private double getSteeringAngleRadiansNoWrap() {
        return steeringMotor.getSelectedSensorPosition() * steeringMotorConversionFactorPosition;
    }

    /**
     * @return the rotation of the wheel
     */
    private Rotation2d getSteeringAngle() {
        return new Rotation2d(Math.IEEEremainder(getSteeringAngleRadiansNoWrap(), Math.PI * 2));
    }

    private double getDriveMotorVelocityMetersPerSecond() {
        return driveMotor.getSelectedSensorVelocity() * driveMotorConversionFactorVelocity;
    }

    public void setDeadModule(boolean isDeadMode) {
        this.isDeadMode = isDeadMode;
        inDeadModeAlert.set(isDeadMode);

        steeringMotor.setNeutralMode(NeutralMode.Coast);
        driveMotor.setNeutralMode(NeutralMode.Coast);
    }

    public CommandBase getToggleDeadModeCommand() {
        return new InstantRunWhenDisabledCommand(() -> setDeadModule(!isDeadMode)).withName("Toggle");
    }

    /**
     * @return the current state of the modules as reported from the encoders
     */
    public SwerveModuleState getActualState() {
        return new SwerveModuleState(getDriveMotorVelocityMetersPerSecond(), getSteeringAngle());
    }

    /**
     * Set the desired state for this swerve module
     *
     * @param state    the desired state
     * @param openLoop if velocity control should be feed forward only. False if to
     *                 use PIDF for velocity control.
     */
    public void setDesiredState(SwerveModuleState state, boolean openLoop) {
        double currentTime = Timer.getFPGATimestamp();
        checkForSteeringMotorReset();
        if (isDeadMode) {
            return;
        }
        // If no speed is requested and the angle is the same as last time
        if (state.speedMetersPerSecond != 0.0 && state.angle.equals(desiredState.angle)) {
            lastMoveTime = currentTime;
        }

        state = SwerveModuleState.optimize(state, getSteeringAngle());

        setDriveReference(state.speedMetersPerSecond, openLoop);
        setAngleReference(state.angle.getRadians());

        desiredState = state;

        // If we have not reset in 5 seconds, been still for 1.5 seconds and our steer
        // velocity is less than half a degree per second (could happen if we are being
        // pushed), reset to absolute
        if (currentTime - lastAbsoluteResetTime > 5.0 && currentTime - lastMoveTime > 1.5
                && absoluteSteeringEncoder.getVelocity() < 0.5) {
            resetSteeringToAbsolute();
        }
    }

    private void setAngleReference(double targetAngleRadians) {
        desiredHeadingEntry.append(targetAngleRadians);
        steeringMotor.set(
                TalonFXControlMode.Position,
                SwerveUtils.calculateContinuousInputSetpoint(getSteeringAngleRadiansNoWrap(), targetAngleRadians)
                        / steeringMotorConversionFactorPosition
        );
    }

    private void setDriveReference(double targetVelocityMetersPerSecond, boolean openLoop) {
        desiredVelocityEntry.append(targetVelocityMetersPerSecond);
        openLoopEntry.append(openLoop);

        if (openLoop) {
            driveMotor.set(TalonFXControlMode.PercentOutput, targetVelocityMetersPerSecond / openLoopMaxSpeed);
        } else {
            // Divide feedforward by 12 because we have to convert from volts to [-1, 1]
            driveMotor.set(
                    TalonFXControlMode.Velocity, targetVelocityMetersPerSecond / driveMotorConversionFactorVelocity,
                    DemandType.ArbitraryFeedForward,
                    driveMotorFF.calculate(targetVelocityMetersPerSecond) / nominalVoltage
            );
        }
    }

    /**
     * Log all telemetry values. Should be called (only) in subsystem periodic
     */
    public void logValues() {
        driveMotor.logValues();
        steeringMotor.logValues();
        actualHeadingEntry.append(getSteeringAngle().getDegrees());
        actualVelocityEntry.append(getDriveMotorVelocityMetersPerSecond());
        absoluteHeadingEntry.append(getAbsoluteDegrees());
        setToAbsoluteEntry.append(setToAbsolute);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Absolute Angle", this::getAbsoluteDegrees, null);
        builder.addDoubleProperty("Read Angle Degrees", () -> getSteeringAngle().getDegrees(), null);
        builder.addDoubleProperty("Drive Velocity", this::getDriveMotorVelocityMetersPerSecond, null);
        builder.addDoubleProperty("Desired Drive Velocity", () -> desiredState.speedMetersPerSecond, null);
        builder.addDoubleProperty("Drive Motor Output Percent", driveMotor::getMotorOutputPercent, null);
        builder.addDoubleProperty("Angle Motor Output Percent", steeringMotor::getMotorOutputPercent, null);
        builder.addDoubleProperty("Desired Angle Degrees", () -> desiredState.angle.getDegrees(), null);
        builder.addBooleanProperty("Is Set to Absolute", () -> setToAbsolute, null);
        builder.addBooleanProperty("In Dead Mode", () -> isDeadMode, null);
    }

    public static class SwerveModuleConfiguration {
        public final int driveMotorPort;
        public final int steeringMotorPort;
        public final int steeringEncoderPort;
        public final boolean driveMotorInverted;
        public final boolean steeringMotorInverted;
        public final double offsetDegrees;
        public final boolean steeringEncoderInverted;
        public final SharedSwerveModuleConfiguration sharedConfiguration;

        public SwerveModuleConfiguration(
                int driveMotorPort, int steeringMotorPort, int steeringEncoderPort, boolean driveMotorInverted,
                boolean steeringMotorInverted, double offsetDegrees, boolean steeringEncoderInverted,
                SharedSwerveModuleConfiguration sharedConfiguration
        ) {
            this.driveMotorPort = driveMotorPort;
            this.steeringMotorPort = steeringMotorPort;
            this.steeringEncoderPort = steeringEncoderPort;
            this.driveMotorInverted = driveMotorInverted;
            this.steeringMotorInverted = steeringMotorInverted;
            this.offsetDegrees = offsetDegrees;
            this.steeringEncoderInverted = steeringEncoderInverted;
            this.sharedConfiguration = sharedConfiguration;
        }

        /**
         * This is all the options that are not module specific
         */
        public static class SharedSwerveModuleConfiguration {
            public final double driveGearRatio;
            public final double steerGearRatio;
            public final double drivePeakCurrentLimit;
            public final double driveContinuousCurrentLimit;
            public final double drivePeakCurrentDurationSeconds;
            public final double steerPeakCurrentLimit;
            public final double steerContinuousCurrentLimit;
            public final double steerPeakCurrentDurationSeconds;
            public final double nominalVoltage;
            public final double wheelDiameterMeters;
            public final double openLoopMaxSpeed;
            public final double angleErrorToleranceRadians;
            public final PIDFFFGains driveVelocityGains;
            public final PIDGains steerPositionGains;

            public SharedSwerveModuleConfiguration(
                    double driveGearRatio, double steerGearRatio, double drivePeakCurrentLimit,
                    double driveContinuousCurrentLimit, double drivePeakCurrentDurationSeconds,
                    double steerPeakCurrentLimit, double steerContinuousCurrentLimit,
                    double steerPeakCurrentDurationSeconds, double nominalVoltage, double wheelDiameterMeters,
                    double openLoopMaxSpeed, PIDFFFGains driveVelocityGains, PIDGains steerPositionGains,
                    double angleErrorToleranceRadians
            ) {
                this.driveGearRatio = driveGearRatio;
                this.steerGearRatio = steerGearRatio;
                this.drivePeakCurrentLimit = drivePeakCurrentLimit;
                this.driveContinuousCurrentLimit = driveContinuousCurrentLimit;
                this.drivePeakCurrentDurationSeconds = drivePeakCurrentDurationSeconds;
                this.steerPeakCurrentLimit = steerPeakCurrentLimit;
                this.steerContinuousCurrentLimit = steerContinuousCurrentLimit;
                this.steerPeakCurrentDurationSeconds = steerPeakCurrentDurationSeconds;
                this.nominalVoltage = nominalVoltage;
                this.wheelDiameterMeters = wheelDiameterMeters;
                this.openLoopMaxSpeed = openLoopMaxSpeed;
                this.angleErrorToleranceRadians = angleErrorToleranceRadians;
                this.driveVelocityGains = driveVelocityGains;
                this.steerPositionGains = steerPositionGains;
            }
        }
    }
}
