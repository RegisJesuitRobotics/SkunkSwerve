package frc.robot.utils;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class SwerveUtilsTest {
    @Test
    void calculateContinuousInputSetpoint_0_0_0() {
        assertEquals(0, SwerveUtils.calculateContinuousInputSetpoint(0, 0));
    }

    @Test
    void calculateContinuousInputSetpoint_2PI_0_2PI() {
        assertEquals(Math.PI * 2, SwerveUtils.calculateContinuousInputSetpoint(Math.PI * 2, 0));
    }

    @Test
    void calculateContinuousInputSetpoint_0_2PI_0() {
        assertEquals(0, SwerveUtils.calculateContinuousInputSetpoint(0, Math.PI * 2));
    }

    @Test
    void calculateContinuousInputSetpoint_3PI_PI_3PI() {
        assertEquals(Math.PI * 3, SwerveUtils.calculateContinuousInputSetpoint(Math.PI * 3, Math.PI));
    }

    @Test
    void applyCircleDeadZone_InsideCircle_NoChange() {
        assertArrayEquals(new double[] { 0.5, 0.5 }, SwerveUtils.applyCircleDeadZone(0.5, 0.5, 1.0));
    }

    @Test
    void applyCircleDeadZone_InsideCircle_NonOneMax_NoChange() {
        assertArrayEquals(new double[] { 1.0, 1.5 }, SwerveUtils.applyCircleDeadZone(1.0, 1.5, 10.0));
    }

    @Test
    void applyCircleDeadZone_OutsideCircle_Normalized() {
        // sqrt(2) / 2
        assertArrayEquals(new double[] { 0.707, 0.707 }, SwerveUtils.applyCircleDeadZone(1.0, 1.0, 1.0), 0.001);
    }

    @Test
    void applyCircleDeadZone_OutsideCircle_Negative_Normalized() {
        assertArrayEquals(new double[] { -0.707, 0.707 }, SwerveUtils.applyCircleDeadZone(-1.0, 1.0, 1.0), 0.001);
    }
}
