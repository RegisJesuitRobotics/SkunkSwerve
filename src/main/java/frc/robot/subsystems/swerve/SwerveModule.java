package frc.robot.subsystems.swerve;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.utils.PIDFFFGains;
import frc.robot.utils.PIDGains;
import frc.robot.utils.SwerveUtils;

public class SwerveModule implements Sendable {
    private static final int CAN_TIMEOUT_MS = 30;
    private static int instances = 0;

    private final DoubleLogEntry desiredVelocityEntry;
    private final DoubleLogEntry desiredHeadingEntry;
    private final DoubleLogEntry actualVelocityEntry;
    private final DoubleLogEntry actualHeadingEntry;
    private final DoubleLogEntry absoluteHeadingEntry;
    private final DoubleLogEntry driveMotorAmpsEntry;
    private final DoubleLogEntry steerMotorAmpsEntry;
    private final StringLogEntry moduleEventEntry;

    private final TalonFX driveMotor;
    private final TalonFX steeringMotor;
    private final CANCoder absoluteSteeringEncoder;

    private final double driveMotorConversionFactorVelocity;
    private final double steeringMotorConversionFactorPosition;
    private final double nominalVoltage;
    private final double steeringEncoderOffset;

    private final SimpleMotorFeedforward driveMotorFF;

    private final double openLoopMaxSpeed;

    private SwerveModuleState desiredState;

    public SwerveModule(SwerveModuleConfiguration config) {
        ++instances;

        // Initialize all logging entries
        DataLog logger = DataLogManager.getLog();
        String tableName = "/drive/modules/" + instances + "/";
        desiredVelocityEntry = new DoubleLogEntry(logger, tableName + "desiredVelocity");
        desiredHeadingEntry = new DoubleLogEntry(logger, tableName + "desiredHeading");
        actualVelocityEntry = new DoubleLogEntry(logger, tableName + "actualVelocity");
        actualHeadingEntry = new DoubleLogEntry(logger, tableName + "actualHeading");
        absoluteHeadingEntry = new DoubleLogEntry(logger, tableName + "absoluteHeading");
        driveMotorAmpsEntry = new DoubleLogEntry(logger, tableName + "driveMotorAmps");
        steerMotorAmpsEntry = new DoubleLogEntry(logger, tableName + "steerMotorAmps");
        moduleEventEntry = new StringLogEntry(logger, tableName + "events");

        // Drive motor
        this.driveMotor = new TalonFX(config.driveMotorPort);
        configDriveMotor(config);

        // Steer encoder
        this.absoluteSteeringEncoder = new CANCoder(config.steeringEncoderPort);
        configSteeringEncoder(config);

        // Steer motor
        this.steeringMotor = new TalonFX(config.steeringMotorPort);
        configSteeringMotor(config);

        this.driveMotorConversionFactorVelocity = (config.sharedConfiguration.wheelDiameterMeters * Math.PI * 10)
                / (config.sharedConfiguration.driveGearRatio * 2048);
        this.steeringMotorConversionFactorPosition = (360) / (config.sharedConfiguration.steerGearRatio * 2048);

        this.nominalVoltage = config.sharedConfiguration.nominalVoltage;
        this.steeringEncoderOffset = config.offsetDegrees;

        this.driveMotorFF = config.sharedConfiguration.driveVelocityGains.feedforward;
        this.openLoopMaxSpeed = config.sharedConfiguration.openLoopMaxSpeed;

        resetSteeringToAbsolute();
    }

    private void configDriveMotor(SwerveModuleConfiguration config) {
        checkCTREError(driveMotor.configFactoryDefault(CAN_TIMEOUT_MS), "Could not config drive motor factory default");

        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

        // Current limit
        motorConfiguration.supplyCurrLimit.currentLimit = config.sharedConfiguration.driveCurrentLimit;
        motorConfiguration.supplyCurrLimit.enable = true;
        // Voltage compensation
        motorConfiguration.voltageCompSaturation = config.sharedConfiguration.nominalVoltage;
        config.sharedConfiguration.driveVelocityGains.setSlot(motorConfiguration.slot0);

        checkCTREError(
                driveMotor.configAllSettings(motorConfiguration, CAN_TIMEOUT_MS), "Could not config drive motor"
        );
        driveMotor.setInverted(
                config.driveMotorInverted ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise
        );

        checkCTREError(
                driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MS),
                "Could not config drive motor sensor"
        );
        driveMotor.setSensorPhase(true);

        driveMotor.setNeutralMode(NeutralMode.Brake);
        checkCTREError(
                driveMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250, CAN_TIMEOUT_MS),
                "Could not config drive status frame"
        );

        moduleEventEntry.append("Drive motor initialized");
    }

    private void configSteeringMotor(SwerveModuleConfiguration config) {
        checkCTREError(steeringMotor.configFactoryDefault(), "Could not config steer motor factory default");

        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

        // Current limit
        motorConfiguration.supplyCurrLimit.currentLimit = config.sharedConfiguration.steerCurrentLimit;
        motorConfiguration.supplyCurrLimit.enable = true;
        // Voltage compensation
        motorConfiguration.voltageCompSaturation = config.sharedConfiguration.nominalVoltage;
        config.sharedConfiguration.steerPositionGains.setSlot(motorConfiguration.slot0);

        checkCTREError(
                steeringMotor.configAllSettings(motorConfiguration, CAN_TIMEOUT_MS), "Could not configure steer motor"
        );
        steeringMotor.setInverted(
                config.steeringMotorInverted ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise
        );

        checkCTREError(
                steeringMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MS),
                "Could not config steer motor sensor"
        );

        // Because + on motor is clockwise, and we want + on encoder to be
        // counter-clockwise we have to set the sensor phase
        steeringMotor.setSensorPhase(true);

        steeringMotor.setNeutralMode(NeutralMode.Brake);

        // We don't really need the information on this status frame, so we can make it
        // not send as often to save CAN bandwidth
        checkCTREError(
                steeringMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250, CAN_TIMEOUT_MS),
                "Could not config steer status frame"
        );

        moduleEventEntry.append("Steer motor initialized");
    }

    private void configSteeringEncoder(SwerveModuleConfiguration config) {
        CANCoderConfiguration encoderConfiguration = new CANCoderConfiguration();

        encoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        encoderConfiguration.sensorDirection = config.steeringEncoderInverted;

        checkCTREError(
                absoluteSteeringEncoder.configAllSettings(encoderConfiguration, CAN_TIMEOUT_MS),
                "Could not configure steer encoder"
        );
        // Because we are only reading this at the beginning we do not have to update it
        // often
        checkCTREError(
                absoluteSteeringEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100, CAN_TIMEOUT_MS),
                "Could not set steer encoder status frame"
        );

        moduleEventEntry.append("Steer encoder initialized");
    }

    private void checkCTREError(ErrorCode errorCode, String message) {
        if (errorCode != ErrorCode.OK) {
            String fullMessage = String.format("%s: %s", message, errorCode.toString());

            moduleEventEntry.append(fullMessage);
            DriverStation.reportError(fullMessage, false);
        }
    }

    /**
     * Resets the integrated encoder on the steering motor to the absolute position
     * of the CANCoder
     */
    public void resetSteeringToAbsolute() {
        setSteerMotorEncoderPosition(getAbsoluteDegrees());
    }

    private void setSteerMotorEncoderPosition(double newPositionDegrees) {
        checkCTREError(
                steeringMotor.setSelectedSensorPosition(
                        newPositionDegrees / steeringMotorConversionFactorPosition, 0, CAN_TIMEOUT_MS
                ), "Could not reset encoder to absolute"
        );

        moduleEventEntry.append("Reset steer motor encoder to position: " + newPositionDegrees);
    }

    private double getAbsoluteDegrees() {
        return absoluteSteeringEncoder.getAbsolutePosition() + steeringEncoderOffset;
    }

    private double getSteeringAngleDegreesNoWrap() {
        return steeringMotor.getSelectedSensorPosition() * steeringMotorConversionFactorPosition;
    }

    /**
     * @return the rotation of the wheel [-180, 180)
     */
    private Rotation2d getSteeringAngle() {
        return Rotation2d.fromDegrees(Math.IEEEremainder(getSteeringAngleDegreesNoWrap(), 360));
    }

    private double getDriveMotorVelocityMetersPerSecond() {
        return driveMotor.getSelectedSensorVelocity() * driveMotorConversionFactorVelocity;
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
        state = SwerveModuleState.optimize(state, getSteeringAngle());
        desiredState = state;

        setDriveReference(state.speedMetersPerSecond, openLoop);
        setAngleReference(state.angle.getDegrees());
    }

    private void setAngleReference(double targetAngleDegrees) {
        desiredHeadingEntry.append(targetAngleDegrees);
        steeringMotor.set(
                TalonFXControlMode.Position,
                SwerveUtils.calculateContinuousInputSetpoint(getSteeringAngleDegreesNoWrap(), targetAngleDegrees)
                        / steeringMotorConversionFactorPosition
        );
    }

    private void setDriveReference(double targetVelocityMetersPerSecond, boolean openLoop) {
        desiredVelocityEntry.append(targetVelocityMetersPerSecond);

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
        actualHeadingEntry.append(getSteeringAngle().getDegrees());
        actualVelocityEntry.append(getDriveMotorVelocityMetersPerSecond());
        absoluteHeadingEntry.append(getAbsoluteDegrees());
        driveMotorAmpsEntry.append(driveMotor.getSupplyCurrent());
        steerMotorAmpsEntry.append(steeringMotor.getSupplyCurrent());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Absolute Angle", this::getAbsoluteDegrees, null);
        builder.addDoubleProperty("Read Angle", () -> getSteeringAngle().getDegrees(), null);
        builder.addDoubleProperty("Drive Velocity", this::getDriveMotorVelocityMetersPerSecond, null);
        builder.addDoubleProperty("Desired Drive Velocity", () -> desiredState.speedMetersPerSecond, null);
        builder.addDoubleProperty("Desired Angle", () -> desiredState.angle.getDegrees(), null);
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
            public final double driveCurrentLimit;
            public final double steerCurrentLimit;
            public final double nominalVoltage;
            public final double wheelDiameterMeters;
            public final double openLoopMaxSpeed;
            public final PIDFFFGains driveVelocityGains;
            public final PIDGains steerPositionGains;

            public SharedSwerveModuleConfiguration(
                    double driveGearRatio, double steerGearRatio, double driveCurrentLimit, double steerCurrentLimit,
                    double nominalVoltage, double wheelDiameterMeters, double openLoopMaxSpeed,
                    PIDFFFGains driveVelocityGains, PIDGains steerPositionGains
            ) {
                this.driveGearRatio = driveGearRatio;
                this.steerGearRatio = steerGearRatio;
                this.driveCurrentLimit = driveCurrentLimit;
                this.steerCurrentLimit = steerCurrentLimit;
                this.nominalVoltage = nominalVoltage;
                this.wheelDiameterMeters = wheelDiameterMeters;
                this.openLoopMaxSpeed = openLoopMaxSpeed;
                this.driveVelocityGains = driveVelocityGains;
                this.steerPositionGains = steerPositionGains;
            }
        }
    }
}
