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

    /**
     * Some controllers are not circular, so they can return something like (1, 1)
     * which has a magnitude of over 1 which could result in requesting too much
     * from the system. This makes sure that nothing goes over the maxMagnitude.
     * Simple example: https://www.desmos.com/calculator/gohl0rmvez
     *
     * @param xValue       the x value
     * @param yValue       the y value
     * @param maxMagnitude the maximum magnitude of the values
     * @return the normalized x and y value. [x, y]
     */
    public static double[] applyCircleDeadZone(double xValue, double yValue, double maxMagnitude) {
        double magnitude = Math.hypot(xValue, yValue);
        if (magnitude > maxMagnitude) {
            return new double[]{ xValue / magnitude * maxMagnitude, yValue / magnitude * maxMagnitude };
        }
        return new double[]{ xValue, yValue };
    }
}
