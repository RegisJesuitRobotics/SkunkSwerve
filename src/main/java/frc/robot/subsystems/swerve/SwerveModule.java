package frc.robot.subsystems.swerve;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.telemetry.TelemetryTalonFX;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.TunablePIDFFFGains;
import frc.robot.utils.TunablePIDGains;
import frc.robot.utils.SwerveUtils;


public class SwerveModule {
    private static final int CAN_TIMEOUT_MS = 250;
    private static final double CANCODER_INITIAL_TIMEOUT_SECONDS = 5.0;

    private static final int CANCODER_PERIOD_MS = 50;

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
    private final Alert steerEncoderFaultAlert;
    private final Alert steerMotorFaultAlert;
    private final Alert driveMotorFaultAlert;
    private final Alert inDeadModeAlert;

    private final TelemetryTalonFX driveMotor;
    private final TelemetryTalonFX steerMotor;
    private final CANCoder absoluteSteerEncoder;

    private final double driveMotorConversionFactorPosition;
    private final double driveMotorConversionFactorVelocity;
    private final double steerMotorConversionFactorPosition;
    private final double steerMotorConversionFactorVelocity;
    private final double nominalVoltage;
    private final double steerEncoderOffset;

    private final TunablePIDFFFGains driveGains;
    private final TunablePIDGains steerGains;
    private SimpleMotorFeedforward driveMotorFF;

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
        notSetToAbsoluteAlert = new Alert(alertPrefix + "Steer is not reset to absolute position", AlertType.ERROR);
        steerEncoderFaultAlert = new Alert(alertPrefix + "Steer encoder had a fault initializing", AlertType.ERROR);
        steerMotorFaultAlert = new Alert(alertPrefix + "Steer motor had a fault initializing", AlertType.ERROR);
        driveMotorFaultAlert = new Alert(alertPrefix + "Drive motor had a fault initializing", AlertType.ERROR);
        inDeadModeAlert = new Alert(alertPrefix + "In dead mode", AlertType.WARNING);

        this.driveMotorConversionFactorPosition = (config.sharedConfiguration.wheelDiameterMeters * Math.PI)
                / (config.sharedConfiguration.driveGearRatio * 2048);
        this.driveMotorConversionFactorVelocity = driveMotorConversionFactorPosition * 10.0;
        this.steerMotorConversionFactorPosition = (Math.PI * 2) / (config.sharedConfiguration.steerGearRatio * 2048);
        this.steerMotorConversionFactorVelocity = steerMotorConversionFactorPosition * 10.0;

        this.driveGains = config.sharedConfiguration.driveVelocityGains;
        this.steerGains = config.sharedConfiguration.steerPositionGains;

        // Drive motor
        this.driveMotor = new TelemetryTalonFX(config.driveMotorPort, tableName + "driveMotor");
        configDriveMotor(config);

        // Steer encoder
        this.absoluteSteerEncoder = new CANCoder(config.steerEncoderPort);
        configSteerEncoder(config);

        // Steer motor
        this.steerMotor = new TelemetryTalonFX(config.steerMotorPort, tableName + "steerMotor");
        configSteerMotor(config);

        this.nominalVoltage = config.sharedConfiguration.nominalVoltage;
        this.steerEncoderOffset = config.offsetDegrees;

        this.driveMotorFF = config.sharedConfiguration.driveVelocityGains.getFeedforward();

        resetSteerToAbsolute(CANCODER_INITIAL_TIMEOUT_SECONDS);
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

    private void configSteerMotor(SwerveModuleConfiguration config) {
        boolean faultInitializing = checkCTREError(
                steerMotor.configFactoryDefault(), "Could not config steer motor factory default"
        );

        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        applyCommonMotorConfiguration(motorConfiguration, config);

        config.sharedConfiguration.steerPositionGains.setSlot(motorConfiguration.slot0);
        motorConfiguration.slot0.allowableClosedloopError = config.sharedConfiguration.angleErrorToleranceRadians
                / steerMotorConversionFactorPosition;

        faultInitializing |= checkCTREError(
                steerMotor.configAllSettings(motorConfiguration, CAN_TIMEOUT_MS), "Could not configure steer motor"
        );

        steerMotor.setInverted(
                config.steerMotorInverted ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise
        );

        faultInitializing |= checkCTREError(
                steerMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MS),
                "Could not config steer motor sensor"
        );

        // Because + on motor is clockwise, and we want + on encoder to be
        // counter-clockwise we have to set the sensor phase
        steerMotor.setSensorPhase(false);

        steerMotor.setNeutralMode(NeutralMode.Brake);

        // Clear the reset of it starting up
        steerMotor.hasResetOccurred();

        steerMotorFaultAlert.set(faultInitializing);
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

    private void configSteerEncoder(SwerveModuleConfiguration config) {
        CANCoderConfiguration encoderConfiguration = new CANCoderConfiguration();

        encoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        encoderConfiguration.sensorDirection = config.steerEncoderInverted;
        encoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;

        boolean faultInitializing = checkCTREError(
                absoluteSteerEncoder.configAllSettings(encoderConfiguration, CAN_TIMEOUT_MS),
                "Could not configure steer encoder"
        );

        // Because we are only reading this at the beginning we do not have to update it
        // often
        faultInitializing |= checkCTREError(
                absoluteSteerEncoder
                        .setStatusFramePeriod(CANCoderStatusFrame.SensorData, CANCODER_PERIOD_MS, CAN_TIMEOUT_MS),
                "Could not set steer encoder status frame"
        );

        steerEncoderFaultAlert.set(faultInitializing);
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

    private void checkForSteerMotorReset() {
        // Steer motor lost power
        if (RobotBase.isReal() && steerMotor.hasResetOccurred()) {
            reportError("Steer motor reset occurred");
            setToAbsolute = false;
            resetSteerToAbsolute();
        }
    }

    /**
     * Resets the integrated encoder on the Steer motor to the absolute position of
     * the CANCoder. Trys only once, and if it fails, it will not try again until
     */
    public void resetSteerToAbsolute() {
        resetSteerToAbsolute(0.0);
    }

    /**
     * Resets the integrated encoder on the Steer motor to the absolute position of
     * the CANCoder
     *
     * @param timeout The timeout in seconds to wait.
     */
    public void resetSteerToAbsolute(double timeout) {
        double startTime = Timer.getFPGATimestamp();
        setToAbsolute = false;

        double absolutePosition;
        boolean gotAbsolutePosition = false;
        do {
            absolutePosition = getAbsoluteDegrees();
            // If no error
            if (!checkCTREError(absoluteSteerEncoder.getLastError(), "Could not get absolute position from encoder")) {
                gotAbsolutePosition = true;
                break;
            }
        } while (timeout != 0.0 || Timer.getFPGATimestamp() - startTime < timeout);

        if (gotAbsolutePosition) {
            ErrorCode settingPositionError = steerMotor.setSelectedSensorPosition(
                    Units.degreesToRadians(absolutePosition) / steerMotorConversionFactorPosition, 0, CAN_TIMEOUT_MS
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
        return Math.IEEEremainder(absoluteSteerEncoder.getAbsolutePosition() + steerEncoderOffset, 360);
    }

    private double getSteerAngleRadiansNoWrap() {
        return steerMotor.getSelectedSensorPosition() * steerMotorConversionFactorPosition;
    }

    /**
     * @return the rotation of the wheel
     */
    private Rotation2d getSteerAngle() {
        return new Rotation2d(getSteerAngleRadiansNoWrap());
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

        steerMotor.setNeutralMode(isDeadMode ? NeutralMode.Coast : NeutralMode.Brake);
        driveMotor.setNeutralMode(isDeadMode ? NeutralMode.Coast : NeutralMode.Brake);
    }

    public void resetDriveMotorPosition() {
        driveMotor.setSelectedSensorPosition(0.0);
    }

    public CommandBase getToggleDeadModeCommand() {
        return Commands.runOnce(() -> setDeadModule(!isDeadMode)).withName("Toggle").ignoringDisable(true);
    }

    /**
     * @return the current state of the modules as reported from the encoders
     */
    public SwerveModuleState getActualState() {
        return new SwerveModuleState(getDriveMotorVelocityMetersPerSecond(), getSteerAngle());
    }

    public SwerveModulePosition getActualPosition() {
        return new SwerveModulePosition(getDriveMotorPositionMeters(), getSteerAngle());
    }

    /**
     * Set the desired state for this swerve module
     *
     * @param state       the desired state
     * @param nextState   the next desired state to be used for acceleration
     *                    feedforward.
     * @param activeSteer if Steer should be active
     * @param openLoop    if velocity control should be feed forward only. False if
     *                    to use PIDF for velocity control.
     */
    public void setDesiredState(
            SwerveModuleState state, SwerveModuleState nextState, boolean activeSteer, boolean openLoop
    ) {
        checkForSteerMotorReset();
        checkAndUpdatePIDGains();
        if (isDeadMode) {
            return;
        }
        double currentTime = Timer.getFPGATimestamp();

        if (state.speedMetersPerSecond != 0.0 || activeSteer) {
            lastMoveTime = currentTime;
        }

        state = SwerveModuleState.optimize(state, getSteerAngle());
        nextState = SwerveModuleState.optimize(nextState, getSteerAngle());

        setDriveReference(state.speedMetersPerSecond, nextState.speedMetersPerSecond, openLoop);
        setAngleReference(Math.IEEEremainder(state.angle.getRadians(), 2 * Math.PI), activeSteer);

        // If we have not reset in 5 seconds, been still for 1.5 seconds and our steer
        // velocity is less than half a degree per second (could happen if we are being
        // pushed), reset to absolute
        if (currentTime - lastAbsoluteResetTime > 5.0 && currentTime - lastMoveTime > 1.5
                && Math.abs(absoluteSteerEncoder.getVelocity()) < 0.5
                && Math.abs(steerMotor.getSelectedSensorVelocity() * steerMotorConversionFactorVelocity) < Units
                        .degreesToRadians(0.5)) {
            resetSteerToAbsolute();
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
            steerMotor.set(
                    TalonFXControlMode.Position,
                    SwerveUtils.calculateContinuousInputSetpoint(getSteerAngleRadiansNoWrap(), targetAngleRadians)
                            / steerMotorConversionFactorPosition
            );
        } else {
            steerMotor.neutralOutput();
        }
    }

    private void setDriveReference(
            double targetVelocityMetersPerSecond, double nextTargetVelocityMetersPerSecond, boolean openLoop
    ) {
        desiredVelocityEntry.append(targetVelocityMetersPerSecond);
        nextDesiredVelocityEntry.append(nextTargetVelocityMetersPerSecond);
        openLoopEntry.append(openLoop);

        double feedforwardValuePercent;
        nextTargetVelocityMetersPerSecond = targetVelocityMetersPerSecond;
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

    private void checkAndUpdatePIDGains() {
        if (driveGains.hasChanged()) {
            SlotConfiguration newSlotConfig = new SlotConfiguration();
            driveGains.setSlot(newSlotConfig);
            driveMotor.configureSlot(newSlotConfig);

            driveMotorFF = driveGains.getFeedforward();

            moduleEventEntry.append("Updated drive PID due to value change");
        }

        if (steerGains.hasChanged()) {
            SlotConfiguration newSlotConfig = new SlotConfiguration();
            steerGains.setSlot(newSlotConfig);
            steerMotor.configureSlot(newSlotConfig);

            moduleEventEntry.append("Updated steer PID due to value change");
        }
    }

    /**
     * Log all telemetry values. Should be called (only) in subsystem periodic
     */
    public void logValues() {
        driveMotor.logValues();
        steerMotor.logValues();
        actualHeadingEntry.append(getSteerAngle().getRadians());
        actualVelocityEntry.append(getDriveMotorVelocityMetersPerSecond());
        absoluteHeadingEntry.append(Units.degreesToRadians(getAbsoluteDegrees()));
        setToAbsoluteEntry.append(setToAbsolute);
    }


    public record SwerveModuleConfiguration(int driveMotorPort, int steerMotorPort, int steerEncoderPort,
            boolean driveMotorInverted, boolean steerMotorInverted, double offsetDegrees, boolean steerEncoderInverted,
            SharedSwerveModuleConfiguration sharedConfiguration) {}

    /**
     * This is all the options that are not module specific
     */
    public record SharedSwerveModuleConfiguration(double driveGearRatio, double steerGearRatio,
            double drivePeakCurrentLimit, double driveContinuousCurrentLimit, double drivePeakCurrentDurationSeconds,
            double steerPeakCurrentLimit, double steerContinuousCurrentLimit, double steerPeakCurrentDurationSeconds,
            double nominalVoltage, double wheelDiameterMeters, double openLoopMaxSpeed,
            TunablePIDFFFGains driveVelocityGains, TunablePIDGains steerPositionGains,
            double angleErrorToleranceRadians) {}
}
