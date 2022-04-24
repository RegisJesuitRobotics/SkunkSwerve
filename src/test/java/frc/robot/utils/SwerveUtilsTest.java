package frc.robot.utils;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class SwerveUtilsTest {
    @Test
    void optimizeAngleSetpoint_Zero_Zero() {
        assertEquals(0, SwerveUtils.calculateContinuousInputSetpoint(0, 0));
    }

    @Test
    void optimizeAngleSetpoint_Zero_Positive_NoWrap() {
        assertEquals(10, SwerveUtils.calculateContinuousInputSetpoint(0, 10));
    }

    @Test
    void optimizeAngleSetpoint_Zero_Negative_NoWrap() {
        assertEquals(-10, SwerveUtils.calculateContinuousInputSetpoint(0, -10));
    }

    @Test
    void optimizeAngleSetpoint_PositiveNotOver_Positive_NoWrap() {
        assertEquals(60, SwerveUtils.calculateContinuousInputSetpoint(50, 60));
    }

    @Test
    void optimizeAngleSetpoint_PositiveNotOver_Negative_NoWrap() {
        assertEquals(-10, SwerveUtils.calculateContinuousInputSetpoint(50, -10));
    }

    @Test
    void optimizeAngleSetpoint_PositiveNotOver_Negative_Wrap() {
        assertEquals(265, SwerveUtils.calculateContinuousInputSetpoint(90, -95));
    }

    @Test
    void optimizeAngleSetpoint_NegativeNotOver_Negative_NoWrap() {
        assertEquals(-60, SwerveUtils.calculateContinuousInputSetpoint(-50, -60));
    }

    @Test
    void optimizeAngleSetpoint_NegativeNotOver_Positive_NoWrap() {
        assertEquals(10, SwerveUtils.calculateContinuousInputSetpoint(-50, 10));
    }

    @Test
    void optimizeAngleSetpoint_NegativeNotOver_Negative_Wrap() {
        assertEquals(-265, SwerveUtils.calculateContinuousInputSetpoint(-90, 95));
    }

    @Test
    void optimizeAngleSetpoint_PositiveHalfOver_Positive_NoWrap() {
        assertEquals(190, SwerveUtils.calculateContinuousInputSetpoint(181, 190));
    }

    @Test
    void optimizeAngleSetpoint_PositiveHalfOver_Positive_Wrap() {
        assertEquals(362, SwerveUtils.calculateContinuousInputSetpoint(185, 2));
    }

    @Test
    void optimizeAngleSetpoint_PositiveOver_Positive_NoWrap() {
        assertEquals(362, SwerveUtils.calculateContinuousInputSetpoint(361, 2));
    }

    @Test
    void optimizeAngleSetpoint_PositiveOver_Negative_NoWrap() {
        assertEquals(719, SwerveUtils.calculateContinuousInputSetpoint(720, -1));
    }

    @Test
    void optimizeAngleSetpoint_PositiveOver_Negative_Wrap() {
        assertEquals(985, SwerveUtils.calculateContinuousInputSetpoint(810, -95));
    }

    @Test
    void optimizeAngleSetpoint_NegativeHalfOver_Negative_NoWrap() {
        assertEquals(-190, SwerveUtils.calculateContinuousInputSetpoint(-181, -190));
    }

    @Test
    void optimizeAngleSetpoint_NegativeHalfOver_Negative_Wrap() {
        assertEquals(-362, SwerveUtils.calculateContinuousInputSetpoint(-185, -2));
    }

    @Test
    void optimizeAngleSetpoint_NegativeOver_Negative_NoWrap() {
        assertEquals(-362, SwerveUtils.calculateContinuousInputSetpoint(-361, -2));
    }

    @Test
    void optimizeAngleSetpoint_NegativeOver_Positive_NoWrap() {
        assertEquals(-719, SwerveUtils.calculateContinuousInputSetpoint(-720, 1));
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
