package frc.robot.utils;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class SwerveMathUtilsTest {
    @Test
    void optimizeAngleSetpoint_Zero_Zero() {
        assertEquals(0, SwerveMathUtils.optimizeAngleSetpoint(0, 0));
    }

    @Test
    void optimizeAngleSetpoint_Zero_Positive_NoWrap() {
        assertEquals(10, SwerveMathUtils.optimizeAngleSetpoint(0, 10));
    }

    @Test
    void optimizeAngleSetpoint_Zero_Negative_NoWrap() {
        assertEquals(-10, SwerveMathUtils.optimizeAngleSetpoint(0, -10));
    }

    @Test
    void optimizeAngleSetpoint_PositiveNotOver_Positive_NoWrap() {
        assertEquals(60, SwerveMathUtils.optimizeAngleSetpoint(50, 60));
    }

    @Test
    void optimizeAngleSetpoint_PositiveNotOver_Negative_NoWrap() {
        assertEquals(-10, SwerveMathUtils.optimizeAngleSetpoint(50, -10));
    }

    @Test
    void optimizeAngleSetpoint_PositiveNotOver_Negative_Wrap() {
        assertEquals(265, SwerveMathUtils.optimizeAngleSetpoint(90, -95));
    }

    @Test
    void optimizeAngleSetpoint_NegativeNotOver_Negative_NoWrap() {
        assertEquals(-60, SwerveMathUtils.optimizeAngleSetpoint(-50, -60));
    }

    @Test
    void optimizeAngleSetpoint_NegativeNotOver_Positive_NoWrap() {
        assertEquals(10, SwerveMathUtils.optimizeAngleSetpoint(-50, 10));
    }

    @Test
    void optimizeAngleSetpoint_NegativeNotOver_Negative_Wrap() {
        assertEquals(-265, SwerveMathUtils.optimizeAngleSetpoint(-90, 95));
    }

    @Test
    void optimizeAngleSetpoint_PositiveHalfOver_Positive_NoWrap() {
        assertEquals(190, SwerveMathUtils.optimizeAngleSetpoint(181, 190));
    }

    @Test
    void optimizeAngleSetpoint_PositiveHalfOver_Positive_Wrap() {
        assertEquals(362, SwerveMathUtils.optimizeAngleSetpoint(185, 2));
    }

    @Test
    void optimizeAngleSetpoint_PositiveOver_Positive_NoWrap() {
        assertEquals(362, SwerveMathUtils.optimizeAngleSetpoint(361, 2));
    }

    @Test
    void optimizeAngleSetpoint_PositiveOver_Negative_NoWrap() {
        assertEquals(719, SwerveMathUtils.optimizeAngleSetpoint(720, -1));
    }

    @Test
    void optimizeAngleSetpoint_PositiveOver_Negative_Wrap() {
        assertEquals(985, SwerveMathUtils.optimizeAngleSetpoint(810, -95));
    }

    @Test
    void optimizeAngleSetpoint_NegativeHalfOver_Negative_NoWrap() {
        assertEquals(-190, SwerveMathUtils.optimizeAngleSetpoint(-181, -190));
    }

    @Test
    void optimizeAngleSetpoint_NegativeHalfOver_Negative_Wrap() {
        assertEquals(-362, SwerveMathUtils.optimizeAngleSetpoint(-185, -2));
    }

    @Test
    void optimizeAngleSetpoint_NegativeOver_Negative_NoWrap() {
        assertEquals(-362, SwerveMathUtils.optimizeAngleSetpoint(-361, -2));
    }

    @Test
    void optimizeAngleSetpoint_NegativeOver_Positive_NoWrap() {
        assertEquals(-719, SwerveMathUtils.optimizeAngleSetpoint(-720, 1));
    }
}
