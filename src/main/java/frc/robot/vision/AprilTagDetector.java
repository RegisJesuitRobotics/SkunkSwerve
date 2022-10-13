package frc.robot.vision;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class AprilTagDetector {
    private static final String TABLE_NAME = "AprilTagDetector";

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable(TABLE_NAME);
    private final NetworkTableEntry calculatedXEntry = table.getEntry("calculatedX");
    private final NetworkTableEntry calculatedYEntry = table.getEntry("calculatedY");
    private final NetworkTableEntry calculatedThetaEntry = table.getEntry("calculatedTheta");
    private final NetworkTableEntry updateCountEntry = table.getEntry("updateCount");

    private final Object resultsLock = new Object();
    private List<AprilTagResult> currentResults;
    private boolean hasNewResults = false;

    public AprilTagDetector() {
        updateCountEntry.addListener(this::processUpdate, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    private void processUpdate(EntryNotification entryNotification) {
        ArrayList<AprilTagResult> newResults = new ArrayList<>();
        Double[] xArray = calculatedXEntry.getDoubleArray(new Double[0]);
        Double[] yArray = calculatedYEntry.getDoubleArray(new Double[0]);
        Double[] thetaArray = calculatedThetaEntry.getDoubleArray(new Double[0]);
        if (xArray.length != yArray.length && xArray.length != thetaArray.length) {
            // TODO: proper error management
            DriverStation.reportError("X, Y, Theta not all equal length (april tag)", false);
            return;
        }

        for (int i = 0; i < xArray.length; i++) {
            // TODO confidence and timestamp
            newResults
                    .add(new AprilTagResult(new Pose2d(xArray[i], yArray[i], new Rotation2d(thetaArray[i])), 0.0, 0.0));
        }

        synchronized (resultsLock) {
            currentResults = Collections.unmodifiableList(newResults);
            hasNewResults = true;
        }
    }

    public boolean hasNewResults() {
        return hasNewResults;
    }

    /**
     * Get the most recent results from the detector. Will set hasNewResults to
     * false.
     *
     * @return the most recent results or null if it is currently being written by
     *         the other thread
     */
    public List<AprilTagResult> getResults() {
        synchronized (resultsLock) {
            hasNewResults = false;
            return currentResults;
        }
    }

    static class AprilTagResult {
        private final Pose2d calculatedPose;
        private final double confidence;
        private final double timestamp;

        AprilTagResult(Pose2d calculatedPose, double confidence, double timestamp) {
            this.calculatedPose = calculatedPose;
            this.confidence = confidence;
            this.timestamp = timestamp;
        }

        public Pose2d getCalculatedPose() {
            return calculatedPose;
        }

        public double getConfidence() {
            return confidence;
        }

        public double getTimestamp() {
            return timestamp;
        }
    }
}
