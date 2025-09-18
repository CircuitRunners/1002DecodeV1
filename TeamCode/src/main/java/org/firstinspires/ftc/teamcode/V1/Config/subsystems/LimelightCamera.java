package org.firstinspires.ftc.teamcode.V1.Config.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimelightCamera {

    public Limelight3A limelightCamera;

    public static double HEADING_KP_TX = 0.02;
    public static double HEADING_KI_TX = 0.0000;
    public static double HEADING_KD_TX = 0.000;
    public static double ROTATION_MIN_POWER = 0.0;
    public double finalRotation;

    LLResult result = limelightCamera.getLatestResult();
    public double error = result.getTxNC();

    public LimelightCamera(HardwareMap hardwareMap) {
        limelightCamera = hardwareMap.get(Limelight3A.class, "limelight");
        //limelight.setPollRateHz(11);
    }
    
    
    public double autoAlign() {

        if (result.isValid()) {
            //finalRotation = (error * HEADING_KP_TX) + (integralSum * HEADING_KI_TX) + (derivative * HEADING_KD_TX);
            finalRotation = (error * HEADING_KP_TX);

            if (Math.abs(finalRotation) > 0 && Math.abs(finalRotation) < ROTATION_MIN_POWER) {
                finalRotation = Math.signum(finalRotation) * ROTATION_MIN_POWER;
            }
        } else {
            finalRotation = 0;
        }
        
        
        return finalRotation;
    }
}
