package org.firstinspires.ftc.teamcode.V1.Config.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimelightCamera {

    public Limelight3A limelightCamera;

    public static double HEADING_KP_TX = 0.02;
    public static double HEADING_KI_TX = 0.0;
    public static double HEADING_KD_TX = 0.0;
    public static double ROTATION_MIN_POWER = 0.0;

    public double finalRotation = 0;
    public double error = 0;

    public LimelightCamera(HardwareMap hardwareMap) {
        limelightCamera = hardwareMap.get(Limelight3A.class, "limelight");
        if (limelightCamera == null) {
            throw new IllegalStateException("❌ Limelight not found in hardwareMap! Check config name 'limelight'.");
        }
    }

    /** Returns the latest LLResult safely */
    public LLResult getResult() {
        LLResult result = limelightCamera.getLatestResult();
        return result;
    }

    /** Updates the error based on the latest tag result */
    public double updateError() {
        LLResult result = getResult();
        if (result != null && result.isValid()) {
            error = result.getTxNC();
        } else {
            error = 0;
        }
        return error;
    }

    /** Auto-align PID rotation value based on limelight data */
    public double autoAlign() {
        // Make sure we are on the correct pipeline - one with digital game field
        limelightCamera.pipelineSwitch(3);

        LLResult result = getResult();
        if (result != null && result.isValid()) {
            error = result.getTxNC();
            finalRotation = error * HEADING_KP_TX;

            // enforce minimum rotation power
            if (Math.abs(finalRotation) > 0 && Math.abs(finalRotation) < ROTATION_MIN_POWER) {
                finalRotation = Math.signum(finalRotation) * ROTATION_MIN_POWER;
            }
        } else {
            finalRotation = 0;
        }
        return finalRotation;
    }

    /** Detects ball order based on AprilTag IDs 21-23; switches pipeline to detection pipeline */
    public BallOrder detectBallOrder() {
        limelightCamera.pipelineSwitch(5); // detection pipeline for ALL april tags

        LLResult result = getResult();
        BallOrder detectedOrder = BallOrder.GREEN_PURPLE_PURPLE; // default

        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                switch (fr.getFiducialId()) {
                    case 21:
                        detectedOrder = BallOrder.GREEN_PURPLE_PURPLE;
                        break;
                    case 22:
                        detectedOrder = BallOrder.PURPLE_GREEN_PURPLE;
                        break;
                    case 23:
                        detectedOrder = BallOrder.PURPLE_PURPLE_GREEN;
                        break;
                    default:
                        // ignore other tags
                        break;
                }
                if (fr.getFiducialId() >= 21 && fr.getFiducialId() <= 23) break; // stop at first relevant tag
            }
        }

        return detectedOrder;
    }

    /** Enum for ball orders */
    public enum BallOrder {
        GREEN_PURPLE_PURPLE, // id 21
        PURPLE_GREEN_PURPLE, // id 22
        PURPLE_PURPLE_GREEN  // id 23
    }
}