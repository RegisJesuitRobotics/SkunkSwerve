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
import frc.robot.utils.PIDFGains;
import frc.robot.utils.SwerveMathUtils;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX steeringMotor;
    private final CANCoder absoluteSteeringEncoder;

    private final double driveMotorConversionFactorVelocity;
    private final double steeringMotorConversionFactorPosition;

    private final double driveArbFF;
    private final double steeringArbFF;

    private boolean optimizeState = true;

    public SwerveModule(SwerveModuleConfiguration config) {
        this.driveMotor = new TalonFX(config.driveMotorPort);
        configDriveMotor(config);

        this.steeringMotor = new TalonFX(config.steeringMotorPort);
        configSteeringMotor(config);

        this.absoluteSteeringEncoder = new CANCoder(config.steeringEncoderPort);
        configSteeringEncoder(config);

        this.driveMotorConversionFactorVelocity = config.driveMotorConversionFactorVelocity;
        this.steeringMotorConversionFactorPosition = config.steeringMotorConversionFactorPosition;

        this.driveArbFF = config.driveVelocityGains.arbFF;
        this.steeringArbFF = config.steeringPositionGains.arbFF;

        resetSteeringToAbsolute();
    }

    public void configDriveMotor(SwerveModuleConfiguration config) {
        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

        // Current limit
        motorConfiguration.supplyCurrLimit.currentLimit = config.driveCurrentLimit;
        motorConfiguration.supplyCurrLimit.enable = true;
        // Voltage compensation
        motorConfiguration.voltageCompSaturation = config.nominalVoltage;
        config.driveVelocityGains.setSlot(motorConfiguration.slot0);

        driveMotor.configAllSettings(motorConfiguration);
        driveMotor.setInverted(
                config.driveMotorInverted ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise);
        driveMotor.setSensorPhase(true);
        driveMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void configSteeringMotor(SwerveModuleConfiguration config) {
        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

        // Current limit
        motorConfiguration.supplyCurrLimit.currentLimit = config.steeringCurrentLimit;
        motorConfiguration.supplyCurrLimit.enable = true;
        // Voltage compensation
        motorConfiguration.voltageCompSaturation = config.nominalVoltage;
        config.steeringPositionGains.setSlot(motorConfiguration.slot0);

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
    public double getSteeringAngleDegrees() {
        return Math.IEEEremainder(getSteeringAngleDegreesNoWrap(), 360);
    }

    public double getDriveMotorVelocityMetersPerSecond() {
        return driveMotor.getSelectedSensorVelocity() * driveMotorConversionFactorVelocity;
    }

    public void setState(SwerveModuleState state) {
        if (optimizeState) {
            state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getSteeringAngleDegrees()));
        }

        setDriveReference(state.speedMetersPerSecond);
        setAngleReference(state.angle.getDegrees());
    }

    public void setOptimizeState(boolean optimizeState) {
        this.optimizeState = optimizeState;
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

    public static class SwerveModuleConfiguration {
        private final int driveMotorPort;
        private final int steeringMotorPort;
        private final int steeringEncoderPort;
        private final double driveCurrentLimit;
        private final double steeringCurrentLimit;
        private final boolean driveMotorInverted;
        private final boolean steeringMotorInverted;
        private final double nominalVoltage;
        private final double offsetDegrees;
        private final boolean steeringEncoderInverted;
        private final PIDFGains driveVelocityGains;
        private final PIDFGains steeringPositionGains;

        // Native units per 100ms to m/s
        private final double driveMotorConversionFactorVelocity;

        // Native units to radians
        private final double steeringMotorConversionFactorPosition;

        public SwerveModuleConfiguration(int driveMotorPort, int steeringMotorPort, int steeringEncoderPort,
                double driveGearRatio, double steeringGearRatio, double driveCurrentLimit, double steeringCurrentLimit,
                boolean driveMotorInverted, boolean steeringMotorInverted, double nominalVoltage, double offsetDegrees,
                boolean steeringEncoderInverted, double wheelDiameterMeters, PIDFGains driveVelocityGains,
                PIDFGains steeringPositionGains) {
            this.driveMotorPort = driveMotorPort;
            this.steeringMotorPort = steeringMotorPort;
            this.steeringEncoderPort = steeringEncoderPort;
            this.driveCurrentLimit = driveCurrentLimit;
            this.steeringCurrentLimit = steeringCurrentLimit;
            this.driveMotorInverted = driveMotorInverted;
            this.steeringMotorInverted = steeringMotorInverted;
            this.nominalVoltage = nominalVoltage;
            this.offsetDegrees = offsetDegrees;
            this.steeringEncoderInverted = steeringEncoderInverted;
            this.driveVelocityGains = driveVelocityGains;
            this.steeringPositionGains = steeringPositionGains;

            this.driveMotorConversionFactorVelocity = (Math.PI * wheelDiameterMeters * 10) / (driveGearRatio * 2048.0);

            // One full rotation is 360 degrees
            this.steeringMotorConversionFactorPosition = (360) / (steeringGearRatio * 2048.0);
        }
    }
}
