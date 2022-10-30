package frc.robot.utils;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveUtils {
    private SwerveUtils() {}

    /**
     * @param currentAngle        what the controller currently reads (radians)
     * @param targetAngleSetpoint the desired angle [-pi, pi)
     * @return the target angle in controller's scope
     */
    public static double calculateContinuousInputSetpoint(double currentAngle, double targetAngleSetpoint) {
        targetAngleSetpoint = Math.IEEEremainder(targetAngleSetpoint, Math.PI * 2);

        double remainder = currentAngle % (Math.PI * 2);
        double adjustedAngleSetpoint = targetAngleSetpoint + (currentAngle - remainder);

        // We don't want to rotate over 180 degrees, so just rotate the other way (add a
        // full rotation)
        if (adjustedAngleSetpoint - currentAngle > Math.PI) {
            adjustedAngleSetpoint -= Math.PI * 2;
        } else if (adjustedAngleSetpoint - currentAngle < -Math.PI) {
            adjustedAngleSetpoint += Math.PI * 2;
        }

        return adjustedAngleSetpoint;
    }

    /**
     * Some controllers are not circular, so they can return something like (1, 1)
     * which has a magnitude of over 1 which could result in requesting too much
     * from the system. This makes sure that nothing goes over the maxMagnitude.
     * Simple example:
     * <a href="https://www.desmos.com/calculator/gohl0rmvez">Desmos</a>
     *
     * @param xValue       the x value
     * @param yValue       the y value
     * @param maxMagnitude the maximum magnitude of the values
     * @return the normalized x and y value. [x, y]
     */
    public static double[] applyCircleDeadZone(double xValue, double yValue, double maxMagnitude) {
        double magnitude = Math.hypot(xValue, yValue);
        if (magnitude > maxMagnitude) {
            return new double[] { xValue / magnitude * maxMagnitude, yValue / magnitude * maxMagnitude };
        }
        return new double[] { xValue, yValue };
    }

    public static boolean inEpoch(ChassisSpeeds chassisSpeeds1, ChassisSpeeds chassisSpeeds2, double epoch) {
        boolean inEpoch = Math.abs(chassisSpeeds1.omegaRadiansPerSecond - chassisSpeeds2.omegaRadiansPerSecond) < epoch;
        inEpoch &= Math.abs(chassisSpeeds1.vxMetersPerSecond - chassisSpeeds2.vxMetersPerSecond) < epoch;
        inEpoch &= Math.abs(chassisSpeeds1.vyMetersPerSecond - chassisSpeeds2.vyMetersPerSecond) < epoch;
        return inEpoch;
    }

    /**
     * @param swerveModuleState the state to copy
     * @return the copied state
     */
    public static SwerveModuleState copySwerveState(SwerveModuleState swerveModuleState) {
        return new SwerveModuleState(swerveModuleState.speedMetersPerSecond, swerveModuleState.angle);
    }
}
