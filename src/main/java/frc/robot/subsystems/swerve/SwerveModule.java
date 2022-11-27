package frc.robot.subsystems.swerve;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.telemetry.TelemetryTalonFX;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.PIDFFFGains;
import frc.robot.utils.PIDGains;
import frc.robot.utils.SwerveUtils;


public class SwerveModule {
    private static final int CAN_TIMEOUT_MS = 250;
    private static final double CANCODER_INITIAL_TIMEOUT_SECONDS = 5.0;

    // FIXME: Update to larger number when needed
    private static final int CANCODER_PERIOD_MS = 19;

    private static int instances = 0;

    private final int instanceId;
    private final DoubleTelemetryEntry desiredVelocityEntry;
    private final DoubleTelemetryEntry nextDesiredVelocityEntry;
    private final BooleanTelemetryEntry openLoopEntry;
    private final DoubleTelemetryEntry desiredHeadingEntry;
    private final BooleanTelemetryEntry activeSteerEntry;
    private final DoubleTelemetryEntry actualVelocityEntry;
    private final DoubleTelemetryEntry actualHeadingEntry;
    private final DoubleTelemetryEntry absoluteHeadingEntry;
    private final BooleanTelemetryEntry setToAbsoluteEntry;
    private final EventTelemetryEntry moduleEventEntry;

    private final Alert notSetToAbsoluteAlert;
    private final Alert steeringEncoderFaultAlert;
    private final Alert steeringMotorFaultAlert;
    private final Alert driveMotorFaultAlert;
    private final Alert inDeadModeAlert;

    private final TelemetryTalonFX driveMotor;
    private final TelemetryTalonFX steeringMotor;
    private final CANCoder absoluteSteeringEncoder;

    private final double driveMotorConversionFactorPosition;
    private final double driveMotorConversionFactorVelocity;
    private final double steeringMotorConversionFactorPosition;
    private final double steeringMotorConversionFactorVelocity;
    private final double nominalVoltage;
    private final double steeringEncoderOffset;

    private final SimpleMotorFeedforward driveMotorFF;

    private boolean setToAbsolute = false;
    private boolean isDeadMode = false;
    private double lastMoveTime = 0.0;
    private double lastAbsoluteResetTime = 0.0;

    /**
     * Constructs a new Swerve Module using the given config
     */
    public SwerveModule(SwerveModuleConfiguration config) {
        instanceId = instances++;

        // Initialize all telemetry entries
        String tableName = "/drive/modules/" + instanceId + "/";
        desiredVelocityEntry = new DoubleTelemetryEntry(tableName + "desiredVelocity", true);
        nextDesiredVelocityEntry = new DoubleTelemetryEntry(tableName + "nextDesiredVelocity", true);
        openLoopEntry = new BooleanTelemetryEntry(tableName + "openLoop", true);
        desiredHeadingEntry = new DoubleTelemetryEntry(tableName + "desiredHeading", true);
        activeSteerEntry = new BooleanTelemetryEntry(tableName + "activeSteer", true);
        actualVelocityEntry = new DoubleTelemetryEntry(tableName + "actualVelocity", true);
        actualHeadingEntry = new DoubleTelemetryEntry(tableName + "actualHeading", true);
        absoluteHeadingEntry = new DoubleTelemetryEntry(tableName + "absoluteHeading", true);
        setToAbsoluteEntry = new BooleanTelemetryEntry(tableName + "setToAbsolute", true);
        moduleEventEntry = new EventTelemetryEntry(tableName + "events");

        String alertPrefix = "Module " + instanceId + ": ";
        notSetToAbsoluteAlert = new Alert(alertPrefix + "Steering is not reset to absolute position", AlertType.ERROR);
        steeringEncoderFaultAlert = new Alert(
                alertPrefix + "Steering encoder had a fault initializing", AlertType.ERROR
        );
        steeringMotorFaultAlert = new Alert(alertPrefix + "Steering motor had a fault initializing", AlertType.ERROR);
        driveMotorFaultAlert = new Alert(alertPrefix + "Drive motor had a fault initializing", AlertType.ERROR);
        inDeadModeAlert = new Alert(alertPrefix + "In dead mode", AlertType.WARNING);

        this.driveMotorConversionFactorPosition = (config.sharedConfiguration.wheelDiameterMeters * Math.PI)
                / (config.sharedConfiguration.driveGearRatio * 2048);
        this.driveMotorConversionFactorVelocity = driveMotorConversionFactorPosition * 10.0;
        this.steeringMotorConversionFactorPosition = (Math.PI * 2) / (config.sharedConfiguration.steerGearRatio * 2048);
        this.steeringMotorConversionFactorVelocity = steeringMotorConversionFactorPosition * 10.0;

        // Drive motor
        this.driveMotor = new TelemetryTalonFX(config.driveMotorPort, tableName + "driveMotor");
        configDriveMotor(config);

        // Steer encoder
        this.absoluteSteeringEncoder = new CANCoder(config.steeringEncoderPort);
        configSteeringEncoder(config);

        // Steer motor
        this.steeringMotor = new TelemetryTalonFX(config.steeringMotorPort, tableName + "steeringMotor");
        configSteeringMotor(config);

        this.nominalVoltage = config.sharedConfiguration.nominalVoltage;
        this.steeringEncoderOffset = config.offsetDegrees;

        this.driveMotorFF = config.sharedConfiguration.driveVelocityGains.feedforward;

        resetSteeringToAbsolute(CANCODER_INITIAL_TIMEOUT_SECONDS);
    }

    private void configDriveMotor(SwerveModuleConfiguration config) {
        boolean faultInitializing = checkCTREError(
                driveMotor.configFactoryDefault(CAN_TIMEOUT_MS), "Could not config drive motor factory default"
        );

        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        applyCommonMotorConfiguration(motorConfiguration, config);

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

        driveMotor.setSensorPhase(false);

        driveMotor.setNeutralMode(NeutralMode.Brake);

        // Clear the reset of it starting up
        driveMotor.hasResetOccurred();

        driveMotorFaultAlert.set(faultInitializing);
        moduleEventEntry.append("Drive motor initialized" + (faultInitializing ? " with faults" : ""));
    }

    private void configSteeringMotor(SwerveModuleConfiguration config) {
        boolean faultInitializing = checkCTREError(
                steeringMotor.configFactoryDefault(), "Could not config steer motor factory default"
        );

        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        applyCommonMotorConfiguration(motorConfiguration, config);

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
        steeringMotor.setSensorPhase(false);

        steeringMotor.setNeutralMode(NeutralMode.Brake);

        // Clear the reset of it starting up
        steeringMotor.hasResetOccurred();

        steeringMotorFaultAlert.set(faultInitializing);
        moduleEventEntry.append("Steer motor initialized" + (faultInitializing ? " with faults" : ""));
    }

    private void applyCommonMotorConfiguration(
            TalonFXConfiguration motorConfiguration, SwerveModuleConfiguration config
    ) {
        // Current limit
        motorConfiguration.supplyCurrLimit.currentLimit = config.sharedConfiguration.driveContinuousCurrentLimit;
        motorConfiguration.supplyCurrLimit.triggerThresholdCurrent = config.sharedConfiguration.drivePeakCurrentLimit;
        motorConfiguration.supplyCurrLimit.triggerThresholdTime = config.sharedConfiguration.drivePeakCurrentDurationSeconds;
        motorConfiguration.supplyCurrLimit.enable = true;

        // Voltage compensation
        motorConfiguration.voltageCompSaturation = config.sharedConfiguration.nominalVoltage;
    }

    private void configSteeringEncoder(SwerveModuleConfiguration config) {
        CANCoderConfiguration encoderConfiguration = new CANCoderConfiguration();

        encoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        encoderConfiguration.sensorDirection = config.steeringEncoderInverted;
        encoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;

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
        return new Rotation2d(getSteeringAngleRadiansNoWrap());
    }

    private double getDriveMotorVelocityMetersPerSecond() {
        return driveMotor.getSelectedSensorVelocity() * driveMotorConversionFactorVelocity;
    }

    private double getDriveMotorPositionMeters() {
        return driveMotor.getSelectedSensorPosition() * driveMotorConversionFactorPosition;
    }

    public void setDeadModule(boolean isDeadMode) {
        this.isDeadMode = isDeadMode;
        inDeadModeAlert.set(isDeadMode);

        steeringMotor.setNeutralMode(isDeadMode ? NeutralMode.Coast : NeutralMode.Brake);
        driveMotor.setNeutralMode(isDeadMode ? NeutralMode.Coast : NeutralMode.Brake);
    }

    public CommandBase getToggleDeadModeCommand() {
        return Commands.runOnce(() -> setDeadModule(!isDeadMode)).withName("Toggle").ignoringDisable(true);
    }

    /**
     * @return the current state of the modules as reported from the encoders
     */
    public SwerveModuleState getActualState() {
        return new SwerveModuleState(getDriveMotorVelocityMetersPerSecond(), getSteeringAngle());
    }

    public SwerveModulePosition getActualPosition() {
        return new SwerveModulePosition(getDriveMotorPositionMeters(), getSteeringAngle());
    }

    /**
     * Set the desired state for this swerve module
     *
     * @param state       the desired state
     * @param nextState   the next desired state to be used for acceleration
     *                    feedforward.
     * @param activeSteer if steering should be active
     * @param openLoop    if velocity control should be feed forward only. False if
     *                    to use PIDF for velocity control.
     */
    public void setDesiredState(
            SwerveModuleState state, SwerveModuleState nextState, boolean activeSteer, boolean openLoop
    ) {
        double currentTime = Timer.getFPGATimestamp();
        checkForSteeringMotorReset();
        if (isDeadMode) {
            return;
        }

        if (state.speedMetersPerSecond != 0.0 || activeSteer) {
            lastMoveTime = currentTime;
        }

        state = SwerveModuleState.optimize(state, getSteeringAngle());

        setDriveReference(state.speedMetersPerSecond, nextState.speedMetersPerSecond, openLoop);
        setAngleReference(Math.IEEEremainder(state.angle.getRadians(), 2 * Math.PI), activeSteer);

        // If we have not reset in 5 seconds, been still for 1.5 seconds and our steer
        // velocity is less than half a degree per second (could happen if we are being
        // pushed), reset to absolute
        if (currentTime - lastAbsoluteResetTime > 5.0 && currentTime - lastMoveTime > 1.5
                && Math.abs(absoluteSteeringEncoder.getVelocity()) < 0.5
                && Math.abs(steeringMotor.getSelectedSensorVelocity() * steeringMotorConversionFactorVelocity) < Units
                        .degreesToRadians(0.5)) {
            resetSteeringToAbsolute();
        }
    }

    public void setCharacterizationVoltage(double voltage) {
        setAngleReference(0.0, true);

        // Divide by the value our voltage compensation is set as
        driveMotor.set(TalonFXControlMode.PercentOutput, voltage / nominalVoltage);
    }

    private void setAngleReference(double targetAngleRadians, boolean activeSteer) {
        activeSteerEntry.append(activeSteer);
        desiredHeadingEntry.append(targetAngleRadians);
        if (activeSteer) {
            steeringMotor.set(
                    TalonFXControlMode.Position,
                    SwerveUtils.calculateContinuousInputSetpoint(getSteeringAngleRadiansNoWrap(), targetAngleRadians)
                            / steeringMotorConversionFactorPosition
            );
        } else {
            steeringMotor.neutralOutput();
        }
    }

    private void setDriveReference(
            double targetVelocityMetersPerSecond, double nextTargetVelocityMetersPerSecond, boolean openLoop
    ) {
        desiredVelocityEntry.append(targetVelocityMetersPerSecond);
        nextDesiredVelocityEntry.append(nextTargetVelocityMetersPerSecond);
        openLoopEntry.append(openLoop);

        double feedforwardValuePercent;
        if (targetVelocityMetersPerSecond == nextTargetVelocityMetersPerSecond) {
            feedforwardValuePercent = driveMotorFF.calculate(targetVelocityMetersPerSecond) / nominalVoltage;
        } else {
            feedforwardValuePercent = driveMotorFF
                    .calculate(targetVelocityMetersPerSecond, nextTargetVelocityMetersPerSecond, 0.02) / nominalVoltage;
        }

        if (openLoop) {
            driveMotor.set(TalonFXControlMode.PercentOutput, feedforwardValuePercent);
        } else {
            driveMotor.set(
                    TalonFXControlMode.Velocity, targetVelocityMetersPerSecond / driveMotorConversionFactorVelocity,
                    DemandType.ArbitraryFeedForward, feedforwardValuePercent
            );
        }
    }

    /**
     * Log all telemetry values. Should be called (only) in subsystem periodic
     */
    public void logValues() {
        driveMotor.logValues();
        steeringMotor.logValues();
        actualHeadingEntry.append(getSteeringAngle().getRadians());
        actualVelocityEntry.append(getDriveMotorVelocityMetersPerSecond());
        absoluteHeadingEntry.append(Units.degreesToRadians(getAbsoluteDegrees()));
        setToAbsoluteEntry.append(setToAbsolute);
    }


    public record SwerveModuleConfiguration(int driveMotorPort, int steeringMotorPort, int steeringEncoderPort,
            boolean driveMotorInverted, boolean steeringMotorInverted, double offsetDegrees,
            boolean steeringEncoderInverted, SharedSwerveModuleConfiguration sharedConfiguration) {}

    /**
     * This is all the options that are not module specific
     */
    public record SharedSwerveModuleConfiguration(double driveGearRatio, double steerGearRatio,
            double drivePeakCurrentLimit, double driveContinuousCurrentLimit, double drivePeakCurrentDurationSeconds,
            double steerPeakCurrentLimit, double steerContinuousCurrentLimit, double steerPeakCurrentDurationSeconds,
            double nominalVoltage, double wheelDiameterMeters, double openLoopMaxSpeed, PIDFFFGains driveVelocityGains,
            PIDGains steerPositionGains, double angleErrorToleranceRadians) {}
}
