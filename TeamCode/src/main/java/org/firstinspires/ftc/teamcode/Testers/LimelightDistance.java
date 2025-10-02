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
package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
@Disabled
@TeleOp(name = "TEST - Limelight3A", group = "Sensor")

public class LimelightDistance extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s", status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();

            double[] pythonOutputs = result.getPythonOutput();

            // Check if the array is valid and has enough elements
            if (pythonOutputs != null && pythonOutputs.length >= 5) {
                double targetDetectedFlag = pythonOutputs[0];
                if (targetDetectedFlag == 1.0) {
                    double targetX = pythonOutputs[1];
                    double targetY = pythonOutputs[2];
                    double targetAngle = pythonOutputs[3];
                    double numContours = pythonOutputs[4];





                    telemetry.addData("Python Target", "X: %.2f, Y: %.2f", targetX, targetY);
                    telemetry.addData("Target Angle", "%.2f", targetAngle);
                    telemetry.addData("Number of Contours", "%d", (int) numContours);
                    // telemetry.addData("ty", "%.2f", Math.toDegrees(targetY));
                    telemetry.addData("Calculated Distance Forward/Back", calculateDistance(targetY));
                    telemetry.addData("Calculated Distance Left/Right (Right should be pos)", calculateLateralOffset(targetX,calculateDistance(targetY)));
                } else {
                    telemetry.addData("Python Output", "No target detected.");
                }
            } else {
                telemetry.addData("Python Output", "No valid data available.");
            }
//

            // The telemetry.update() must be here to be called on every loop
            telemetry.update();
        }

        // limelight.stop() is called after the op mode ends
        limelight.stop();
    }



    public double calculateDistance(double targetY) {
        // Variable for the height of the camera above the floor in inches
        double cameraHeight = 6.5;

        //  how many degrees is limelight rotated back from perfecetly vertical
        double cameraMountAngle = 0;

        // Constant for the height of the sample in inches
        double sampleHeight = 0.75;

        // The targetY value from your Python script, representing the vertical angle to the target
        double targetAngle = targetY;

        // Convert angles to radians for Java's trigonometric functions
        double cameraMountAngleRad = Math.toRadians(cameraMountAngle);
        double targetAngleRad = Math.toRadians(targetAngle);

        // Calculate the distance using the formula d = (h2 - h1) / tan(a1 + a2)
        double distance = (sampleHeight - cameraHeight) / Math.tan(cameraMountAngleRad + targetAngleRad)-8;

        return (distance * -1);
    }


    public double calculateLateralOffset(double tx, double forwardDistance) {
        // Convert horizontal angle to radians
        double txRad = Math.toRadians(tx);

        // Horizontal offset = tan(angle) * forward distance
        double lateralOffset = Math.tan(txRad) * forwardDistance;

        //returns in inches
        return lateralOffset;  // positive = right, negative = left
    }


}



