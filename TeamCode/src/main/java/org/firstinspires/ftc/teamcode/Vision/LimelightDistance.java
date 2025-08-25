/*
Copyright (c) 2024 Limelight Vision

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/*
 * This OpMode illustrates how to use the Limelight3A Vision Sensor.
 *
 * @see <a href="https://limelightvision.io/">Limelight</a>
 *
 * Notes on configuration:
 *
 *   The device presents itself, when plugged into a USB port on a Control Hub as an ethernet
 *   interface.  A DHCP server running on the Limelight automatically assigns the Control Hub an
 *   ip address for the new ethernet interface.
 *
 *   Since the Limelight is plugged into a USB port, it will be listed on the top level configuration
 *   activity along with the Control Hub Portal and other USB devices such as webcams.  Typically
 *   serial numbers are displayed below the device's names.  In the case of the Limelight device, the
 *   Control Hub's assigned ip address for that ethernet interface is used as the "serial number".
 *
 *   Tapping the Limelight's name, transitions to a new screen where the user can rename the Limelight
 *   and specify the Limelight's ip address.  Users should take care not to confuse the ip address of
 *   the Limelight itself, which can be configured through the Limelight settings page via a web browser,
 *   and the ip address the Limelight device assigned the Control Hub and which is displayed in small text
 *   below the name of the Limelight on the top level configuration screen.
 */
@TeleOp(name = "TEST - Limelight3A", group = "Sensor")

public class LimelightDistance extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);  // switch for different colors - 0 for yellow, 1 for red, 2 for blue

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                double[] pythonOutputs = result.getPythonOutput();

                // Check if the array is valid and has enough elements
                if (pythonOutputs != null && pythonOutputs.length >= 5) {

                    // Get the values you want
                    double targetDetectedFlag = pythonOutputs[0]; // 1.0 if target is detected
                    double targetX = pythonOutputs[1];            // X-coordinate of the target
                    double targetY = pythonOutputs[2];            // Y-coordinate of the target
                    double targetAngle = pythonOutputs[3];        // Angle of the target
                    double numContours = pythonOutputs[4];        // Number of detected contours

                    // Check if a target was actually detected (based on your Python code's flag)
                    if (targetDetectedFlag == 1.0) {
                        telemetry.addData("Python Target", "X: %.2f, Y: %.2f", targetX, targetY);
                        telemetry.addData("Target Angle", "%.2f", targetAngle);
                        telemetry.addData("Number of Contours", "%d", (int) numContours);

                        // Use targetY for your distance calculation
                        telemetry.addData("Calculated Distance", calculateDistance(targetY));
                    } else {
                        telemetry.addData("Python Output", "No target detected.");
                    }
                } else {
                    telemetry.addData("Python Output", "No valid data available.");
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }

//                // alternate untested (prob worse ngl) logic for getting from logic tables ↓
//                if (result.getPythonOutput() != null ) {
//                    // Access specific values by their index
//                    double targetX = result.getPythonOutput()[0];
//                    double targetY = result.getPythonOutput()[1];
//                    double targetWidth = result.getPythonOutput()[2];
//                    double targetHeight = result.getPythonOutput()[3];
//                    double confidence = result.getPythonOutput()[4];


                    // Now you can use these variables for other tasks,
                    // such as driving the robot
                   // if (confidence > 0.5) { // Only use the data if confidence is high enough




//                telemetry.addData("tx", result.getTx());
//                telemetry.addData("txnc", result.getTxNC());
//                telemetry.addData("ty", result.getTy());
//                telemetry.addData("tync", result.getTyNC());

                // telemetry.addData("Botpose", botpose.toString());

//                // Access barcode results
//                List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
//                for (LLResultTypes.BarcodeResult br : barcodeResults) {
//                    telemetry.addData("Barcode", "Data: %s", br.getData());
//                }
//
//                // Access classifier results
//                List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
//                for (LLResultTypes.ClassifierResult cr : classifierResults) {
//                    telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
//                }

//                // Access detector results
//                List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
//                for (LLResultTypes.DetectorResult dr : detectorResults) {
//                    telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
//                    telemetry.addData("target x degrees", dr.getTargetXDegrees());
//                    telemetry.addData("target y degrees", dr.getTargetYDegrees());
//                    telemetry.addData("Confidence", dr.getConfidence());
//
//
//
//
//                }
//
//                // Access fiducial results
//                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
//                for (LLResultTypes.FiducialResult fr : fiducialResults) {
//                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
//                }
//
//                // Access color results
//                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
//                for (LLResultTypes.ColorResult cr : colorResults) {
//                    telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
//                }
//            } else {
//                telemetry.addData("Limelight", "No data available");
            }
        telemetry.update();

        limelight.stop();
    }



    public double calculateDistance(double targetY) {
        // Variable for the height of the camera above the floor in inches
        double cameraHeight = 6.0;

        // Variable for the angle the camera is mounted at in degrees
        double cameraMountAngle = 45.0;

        // Constant for the height of the sample in inches
        double sampleHeight = 1.5;

        // The targetY value from your Python script, representing the vertical angle to the target
        double targetAngle = targetY;

        // Convert angles to radians for Java's trigonometric functions
        double cameraMountAngleRad = Math.toRadians(cameraMountAngle);
        double targetAngleRad = Math.toRadians(targetAngle);

        // Calculate the distance using the formula d = (h2 - h1) / tan(a1 + a2)
        double distance = (sampleHeight - cameraHeight) / Math.tan(cameraMountAngleRad + targetAngleRad);

        return distance;
    }


}



