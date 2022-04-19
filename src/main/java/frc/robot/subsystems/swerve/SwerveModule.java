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
import frc.robot.utils.SwerveMathUtils;

public class SwerveModule implements Sendable {
    private static int instances = 0;

    private final DoubleLogEntry desiredVelocityEntry;
    private final DoubleLogEntry desiredHeadingEntry;
    private final DoubleLogEntry actualVelocityEntry;
    private final DoubleLogEntry actualHeadingEntry;
    private final DoubleLogEntry driveMotorAmpsEntry;
    private final DoubleLogEntry steerMotorAmpsEntry;
    private final StringLogEntry moduleEventEntry;

    private final TalonFX driveMotor;
    private final TalonFX steeringMotor;
    private final CANCoder absoluteSteeringEncoder;

    private final double driveMotorConversionFactorVelocity;
    private final double steeringMotorConversionFactorPosition;
    private final double nominalVoltage;

    private final SimpleMotorFeedforward driveMotorFF;

    private final double openLoopMaxSpeed;

    private SwerveModuleState desiredState;

    public SwerveModule(SwerveModuleConfiguration config) {
        ++instances;

        DataLog logger = DataLogManager.getLog();
        String tableName = "/drive/modules/" + instances + "/";
        desiredVelocityEntry = new DoubleLogEntry(logger, tableName + "desiredVelocity");
        desiredHeadingEntry = new DoubleLogEntry(logger, tableName + "desiredHeading");
        actualVelocityEntry = new DoubleLogEntry(logger, tableName + "actualVelocity");
        actualHeadingEntry = new DoubleLogEntry(logger, tableName + "actualHeading");
        driveMotorAmpsEntry = new DoubleLogEntry(logger, tableName + "driveMotorAmps");
        steerMotorAmpsEntry = new DoubleLogEntry(logger, tableName + "steerMotorAmps");
        moduleEventEntry = new StringLogEntry(logger, tableName + "events");

        this.driveMotor = new TalonFX(config.driveMotorPort);
        configDriveMotor(config);

        this.absoluteSteeringEncoder = new CANCoder(config.steeringEncoderPort);
        configSteeringEncoder(config);

        this.steeringMotor = new TalonFX(config.steeringMotorPort);
        configSteeringMotor(config);

        this.driveMotorConversionFactorVelocity = (config.sharedConfiguration.wheelDiameterMeters * Math.PI * 10)
                / (config.sharedConfiguration.driveGearRatio * 2048);
        this.steeringMotorConversionFactorPosition = (360) / (config.sharedConfiguration.steerGearRatio * 2048);

        this.nominalVoltage = config.sharedConfiguration.nominalVoltage;

        this.driveMotorFF = config.sharedConfiguration.driveVelocityGains.feedforward;
        this.openLoopMaxSpeed = config.sharedConfiguration.openLoopMaxSpeed;

        resetSteeringToAbsolute();
    }

    public void configDriveMotor(SwerveModuleConfiguration config) {
        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

        // Current limit
        motorConfiguration.supplyCurrLimit.currentLimit = config.sharedConfiguration.driveCurrentLimit;
        motorConfiguration.supplyCurrLimit.enable = true;
        // Voltage compensation
        motorConfiguration.voltageCompSaturation = config.sharedConfiguration.nominalVoltage;
        config.sharedConfiguration.driveVelocityGains.setSlot(motorConfiguration.slot0);

        checkCTREError(driveMotor.configAllSettings(motorConfiguration), "Could not config drive motor");
        driveMotor.setInverted(
                config.driveMotorInverted ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise
        );
        checkCTREError(
                driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor),
                "Could not config drive motor sensor"
        );
        driveMotor.setSensorPhase(true);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        checkCTREError(
                driveMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250),
                "Could not config drive status frame"
        );

        moduleEventEntry.append("Drive motor initialized");
    }

    public void configSteeringMotor(SwerveModuleConfiguration config) {
        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

        // Current limit
        motorConfiguration.supplyCurrLimit.currentLimit = config.sharedConfiguration.steerCurrentLimit;
        motorConfiguration.supplyCurrLimit.enable = true;
        // Voltage compensation
        motorConfiguration.voltageCompSaturation = config.sharedConfiguration.nominalVoltage;
        config.sharedConfiguration.steerPositionGains.setSlot(motorConfiguration.slot0);

        checkCTREError(steeringMotor.configAllSettings(motorConfiguration), "Could not configure steer motor");
        steeringMotor.setInverted(
                config.steeringMotorInverted ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise
        );
        checkCTREError(
                steeringMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor),
                "Could not config steer motor sensor"
        );
        steeringMotor.setSensorPhase(true);
        steeringMotor.setNeutralMode(NeutralMode.Brake);

        // We don't really need the information on this status frame, so we can make it
        // not send as often to save CAN bandwidth
        checkCTREError(
                steeringMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250),
                "Could not config steer status frame"
        );

        moduleEventEntry.append("Steer motor initialized");
    }

    public void configSteeringEncoder(SwerveModuleConfiguration config) {
        CANCoderConfiguration encoderConfiguration = new CANCoderConfiguration();

        encoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        encoderConfiguration.magnetOffsetDegrees = config.offsetDegrees;
        encoderConfiguration.sensorDirection = config.steeringEncoderInverted;
        encoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;

        checkCTREError(
                absoluteSteeringEncoder.configAllSettings(encoderConfiguration), "Could not configure steer encoder"
        );
        // Because we are only reading this at the beginning we do not have to update it
        // often
        checkCTREError(
                absoluteSteeringEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100),
                "Could not set steer encoder status frame"
        );
        absoluteSteeringEncoder.setPositionToAbsolute();

        moduleEventEntry.append("Steer encoder initialized");
    }

    private void checkCTREError(ErrorCode errorCode, String message) {
        if (errorCode != ErrorCode.OK) {
            String fullMessage = String.format("%s: %s", message, errorCode.toString());

            moduleEventEntry.append(fullMessage);
            DriverStation.reportError(fullMessage, false);
        }
    }

    public void resetSteeringToAbsolute() {
        double absolutePosition = absoluteSteeringEncoder.getAbsolutePosition();

        checkCTREError(
                steeringMotor.setSelectedSensorPosition(absolutePosition / steeringMotorConversionFactorPosition),
                "Could not reset encoder to absolute"
        );

        moduleEventEntry.append("Reset steer motor encoder to absolute position: " + absolutePosition);
    }

    private double getSteeringAngleDegreesNoWrap() {
        return steeringMotor.getSelectedSensorPosition() * steeringMotorConversionFactorPosition;
    }

    private double getSteeringAngleDegrees() {
        return Math.IEEEremainder(getSteeringAngleDegreesNoWrap(), 360);
    }

    /**
     * @return the rotation of the wheel [-180, 180)
     */
    private Rotation2d getSteeringAngle() {
        return Rotation2d.fromDegrees(getSteeringAngleDegrees());
    }

    private double getDriveMotorVelocityMetersPerSecond() {
        return driveMotor.getSelectedSensorVelocity() * driveMotorConversionFactorVelocity;
    }

    public SwerveModuleState getActualState() {
        return new SwerveModuleState(getDriveMotorVelocityMetersPerSecond(), getSteeringAngle());
    }

    public void setDesiredState(SwerveModuleState state, boolean openLoop) {
        state = SwerveModuleState.optimize(state, getSteeringAngle());
        desiredState = state;

        setDriveReference(state.speedMetersPerSecond, openLoop);
        setAngleReference(state.angle.getDegrees());
    }

    public void logValues() {
        actualHeadingEntry.append(getSteeringAngleDegrees());
        actualVelocityEntry.append(getDriveMotorVelocityMetersPerSecond());
        driveMotorAmpsEntry.append(driveMotor.getSupplyCurrent());
        steerMotorAmpsEntry.append(steeringMotor.getSupplyCurrent());
    }

    private void setAngleReference(double targetAngleDegrees) {
        desiredHeadingEntry.append(targetAngleDegrees);
        steeringMotor.set(
                TalonFXControlMode.Position,
                SwerveMathUtils.optimizeAngleSetpoint(
                        getSteeringAngleDegreesNoWrap(), targetAngleDegrees / steeringMotorConversionFactorPosition
                )
        );
    }

    private void setDriveReference(double targetVelocityMetersPerSecond, boolean openLoop) {
        desiredVelocityEntry.append(targetVelocityMetersPerSecond);

        if (openLoop) {
            driveMotor.set(TalonFXControlMode.PercentOutput, targetVelocityMetersPerSecond / openLoopMaxSpeed);
        } else {
            // Divide by 12 because we have to convert from volts to
            driveMotor.set(
                    TalonFXControlMode.Velocity, targetVelocityMetersPerSecond / driveMotorConversionFactorVelocity,
                    DemandType.ArbitraryFeedForward,
                    driveMotorFF.calculate(targetVelocityMetersPerSecond) / nominalVoltage
            );
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Absolute Angle", absoluteSteeringEncoder::getAbsolutePosition, null);
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
