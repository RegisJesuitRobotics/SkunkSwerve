package frc.robot.utils;

public class SwerveMathUtils {
    private SwerveMathUtils() {}

    public static double optimizeAngleSetpoint(double currentAngle, double targetAngleSetpoint) {
        targetAngleSetpoint = Math.IEEEremainder(targetAngleSetpoint, 360);

        double remainder = currentAngle % 360;
        double adjustedAngleSetpoint = targetAngleSetpoint + (currentAngle - remainder);

        // We don't want to rotate over 180 degrees, so just rotate the other way
        if (targetAngleSetpoint - remainder > 180.0) {
            adjustedAngleSetpoint -= 360;
        } else if (targetAngleSetpoint - remainder < -180.0) {
            adjustedAngleSetpoint += 360;
        }

        return adjustedAngleSetpoint;
    }
}
