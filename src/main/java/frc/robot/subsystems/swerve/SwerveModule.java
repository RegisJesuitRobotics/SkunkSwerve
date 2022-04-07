package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.utils.PIDFGains;
import frc.robot.utils.SwerveMathUtils;

public class SwerveModule implements Sendable {
    private final TalonFX driveMotor;
    private final TalonFX steeringMotor;
    private final CANCoder absoluteSteeringEncoder;

    private final double driveMotorConversionFactorVelocity;
    private final double steeringMotorConversionFactorPosition;

    private final double driveArbFF;
    private final double steeringArbFF;

    private SwerveModuleState desiredState;

    public SwerveModule(SwerveModuleConfiguration config) {
        this.driveMotor = new TalonFX(config.driveMotorPort);
        configDriveMotor(config);

        this.steeringMotor = new TalonFX(config.steeringMotorPort);
        configSteeringMotor(config);

        this.absoluteSteeringEncoder = new CANCoder(config.steeringEncoderPort);
        configSteeringEncoder(config);

        this.driveMotorConversionFactorVelocity = (config.sharedSwerveModuleConfiguration.wheelDiameterMeters * Math.PI
                * 10) / (config.sharedSwerveModuleConfiguration.driveGearRatio * 2048);
        this.steeringMotorConversionFactorPosition = (360)
                / (config.sharedSwerveModuleConfiguration.steerGearRatio * 2048);

        this.driveArbFF = config.sharedSwerveModuleConfiguration.driveVelocityGains.arbFF;
        this.steeringArbFF = config.sharedSwerveModuleConfiguration.steerPositionGains.arbFF;

        resetSteeringToAbsolute();
    }

    public void configDriveMotor(SwerveModuleConfiguration config) {
        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

        // Current limit
        motorConfiguration.supplyCurrLimit.currentLimit = config.sharedSwerveModuleConfiguration.driveCurrentLimit;
        motorConfiguration.supplyCurrLimit.enable = true;
        // Voltage compensation
        motorConfiguration.voltageCompSaturation = config.sharedSwerveModuleConfiguration.nominalVoltage;
        config.sharedSwerveModuleConfiguration.driveVelocityGains.setSlot(motorConfiguration.slot0);

        driveMotor.configAllSettings(motorConfiguration);
        driveMotor.setInverted(
                config.driveMotorInverted ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise);
        driveMotor.setSensorPhase(true);
        driveMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void configSteeringMotor(SwerveModuleConfiguration config) {
        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

        // Current limit
        motorConfiguration.supplyCurrLimit.currentLimit = config.sharedSwerveModuleConfiguration.steerCurrentLimit;
        motorConfiguration.supplyCurrLimit.enable = true;
        // Voltage compensation
        motorConfiguration.voltageCompSaturation = config.sharedSwerveModuleConfiguration.nominalVoltage;
        config.sharedSwerveModuleConfiguration.steerPositionGains.setSlot(motorConfiguration.slot0);

        steeringMotor.configAllSettings(motorConfiguration);
        steeringMotor.setInverted(
                config.steeringMotorInverted ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise);
        steeringMotor.setSensorPhase(true);
        steeringMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void configSteeringEncoder(SwerveModuleConfiguration config) {
        CANCoderConfiguration encoderConfiguration = new CANCoderConfiguration();

        encoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        encoderConfiguration.magnetOffsetDegrees = config.offsetDegrees;
        encoderConfiguration.sensorDirection = config.steeringEncoderInverted;
        encoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;

        absoluteSteeringEncoder.configAllSettings(encoderConfiguration);
        absoluteSteeringEncoder.setPositionToAbsolute();
    }

    public void resetSteeringToAbsolute() {
        steeringMotor.setSelectedSensorPosition(
                absoluteSteeringEncoder.getAbsolutePosition() / steeringMotorConversionFactorPosition);
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

    public SwerveModuleState getActualState() {
        return new SwerveModuleState(getDriveMotorVelocityMetersPerSecond(), getSteeringAngle());
    }

    public void setDesiredState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getSteeringAngle());
        desiredState = state;

        setDriveReference(state.speedMetersPerSecond);
        setAngleReference(state.angle.getDegrees());
    }

    private void setAngleReference(double targetAngleDegrees) {
        steeringMotor.set(TalonFXControlMode.Position,
                SwerveMathUtils.optimizeAngleSetpoint(getSteeringAngleDegreesNoWrap(),
                        targetAngleDegrees / steeringMotorConversionFactorPosition),
                DemandType.ArbitraryFeedForward, steeringArbFF);
    }

    private void setDriveReference(double targetVelocityMetersPerSecond) {
        driveMotor.set(TalonFXControlMode.Velocity, targetVelocityMetersPerSecond / driveMotorConversionFactorVelocity,
                DemandType.ArbitraryFeedForward, driveArbFF);
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
        public final SharedSwerveModuleConfiguration sharedSwerveModuleConfiguration;

        public SwerveModuleConfiguration(int driveMotorPort, int steeringMotorPort, int steeringEncoderPort,
                boolean driveMotorInverted, boolean steeringMotorInverted, double offsetDegrees,
                boolean steeringEncoderInverted, SharedSwerveModuleConfiguration sharedSwerveModuleConfiguration) {
            this.driveMotorPort = driveMotorPort;
            this.steeringMotorPort = steeringMotorPort;
            this.steeringEncoderPort = steeringEncoderPort;
            this.driveMotorInverted = driveMotorInverted;
            this.steeringMotorInverted = steeringMotorInverted;
            this.offsetDegrees = offsetDegrees;
            this.steeringEncoderInverted = steeringEncoderInverted;
            this.sharedSwerveModuleConfiguration = sharedSwerveModuleConfiguration;
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
            public final PIDFGains driveVelocityGains;
            public final PIDFGains steerPositionGains;

            public SharedSwerveModuleConfiguration(double driveGearRatio, double steerGearRatio,
                    double driveCurrentLimit, double steerCurrentLimit, double nominalVoltage,
                    double wheelDiameterMeters, PIDFGains driveVelocityGains, PIDFGains steerPositionGains) {
                this.driveGearRatio = driveGearRatio;
                this.steerGearRatio = steerGearRatio;
                this.driveCurrentLimit = driveCurrentLimit;
                this.steerCurrentLimit = steerCurrentLimit;
                this.nominalVoltage = nominalVoltage;
                this.wheelDiameterMeters = wheelDiameterMeters;
                this.driveVelocityGains = driveVelocityGains;
                this.steerPositionGains = steerPositionGains;
            }
        }
    }
}
