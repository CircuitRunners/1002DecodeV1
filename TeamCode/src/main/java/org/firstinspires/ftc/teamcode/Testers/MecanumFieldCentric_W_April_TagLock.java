package org.firstinspires.ftc.teamcode.Testers;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.MemDevDecode.Config.MecanumDrive;

import java.util.List;
import java.util.Locale;

@Configurable
@TeleOp(name = "Field Centric With April Tags Lock")
public class MecanumFieldCentric_W_April_TagLock extends OpMode {



    private MecanumDrive drive;


    private GamepadEx player1;
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;

    // PID constants are public and static for Panels UI
    public static double HEADING_KP_TX = 0.02;
    public static double HEADING_KI_TX = 0.0000;
    public static double HEADING_KD_TX = 0.000;
    public static double ROTATION_MIN_POWER = 0.0;

    // PID state variables
    private double integralSum = 0;
    private double lastError = 0;
    private final ElapsedTime timer = new ElapsedTime();

    // State variable to manage the heading lock.
    private boolean isHeadingLocked = false;


    @Override
    public void init(){
        telemetry.addLine("Initializing...");
        telemetry.update();

        player1 = new GamepadEx(gamepad1);

        drive = new MecanumDrive();
        drive.init(hardwareMap);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configurePinpoint();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);
        limelight.start();

        telemetry.addLine("Ready!");
        telemetry.update();
    }

    @Override
    public void loop() {

        player1.readButtons();

        double forward = player1.getLeftY();
        double strafe  =  player1.getLeftX();
        double rotate  =  player1.getRightX();

        // Toggle the heading lock with dpad up/down
//        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
//            isHeadingLocked = true;
//            timer.reset();
//            integralSum = 0;
//            lastError = 0;
//            telemetry.addLine("Heading Lock Enabled.");
//        }
//        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
//            isHeadingLocked = false;
//            telemetry.addLine("Heading Lock Disabled.");
//        }

        pinpoint.update();
        Pose2D currentPose = pinpoint.getPosition();

        double robotHeading = Math.toRadians(currentPose.getHeading(AngleUnit.DEGREES));
        double finalRotation;

        LLResult result = limelight.getLatestResult();

        if (player1.isDown(GamepadKeys.Button.DPAD_UP)) {

            int[] validIDs = {3, 4};

            //limelight.updateRobotOrientation(0.0);
            telemetry.addData("MT2", result.getBotpose_MT2().toString());
            telemetry.addData("Botpose mt2 x", result.getBotpose_MT2().getPosition().x);
            telemetry.addData("Botpose mt2 y" , result.getBotpose_MT2().getPosition().y);
            telemetry.addData("Botpose mt2 z", result.getBotpose_MT2().getPosition().z);


            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                if (result.isValid() && ((fr.getFiducialId() == 20) || (fr.getFiducialId() == 24))){
                    pinpoint.setPosX(result.getBotpose_MT2().getPosition().x + 72, DistanceUnit.INCH);
                    pinpoint.setPosY(result.getBotpose_MT2().getPosition().y + 72, DistanceUnit.INCH);
                }

            }


//        } if (player1.isDown(GamepadKeys.Button.DPAD_DOWN)) {
//
//            int[] validIDs = {3, 4};
//
//            //limelight.updateRobotOrientation(0.0);
//            telemetry.addData("MT2", result.getBotpose_MT2().toString());
//            telemetry.addData("Botpose mt2 x", result.getBotpose_MT2().getPosition().x);
//            telemetry.addData("Botpose mt2 y" , result.getBotpose_MT2().getPosition().y);
//            telemetry.addData("Botpose mt2 z", result.getBotpose_MT2().getPosition().z);
//
//            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
//            for (LLResultTypes.FiducialResult fr : fiducialResults) {
//                if (result.isValid() && ((fr.getFiducialId() == 20) || (fr.getFiducialId() == 24))){
//                    pinpoint.setPosition(result.getBotpose_MT2().getPosition().x + 72,result.getBotpose_MT2().getPosition().y + 72,currentPose.getHeading(AngleUnit.DEGREES));
//                }
//
//            }
//

        }

        if (isHeadingLocked) {


            double error = result.getTxNC();


            if (result.isValid()) {
                //finalRotation = (error * HEADING_KP_TX) + (integralSum * HEADING_KI_TX) + (derivative * HEADING_KD_TX);
                finalRotation = (error * HEADING_KP_TX);

                if (Math.abs(finalRotation) > 0 && Math.abs(finalRotation) < ROTATION_MIN_POWER) {
                    finalRotation = Math.signum(finalRotation) * ROTATION_MIN_POWER;
                }
            }
            else{
                finalRotation = 0;
            }

            telemetry.addData("Using Limelight Data for Heading Lock.", "");
            telemetry.addData("PID Error", error);
            telemetry.addData("PID Rotation", finalRotation);

        }
        else {

            finalRotation = rotate;
            telemetry.addData("Using Pinpoint IMU Heading.", "");
        }

        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(forward, strafe);
        theta = AngleUnit.normalizeRadians(theta - robotHeading);

        double newForward = r * Math.sin(theta);
        double newStrafe  = r * Math.cos(theta);

        drive.drive(newForward, newStrafe, finalRotation);

        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) {
            pinpoint.resetPosAndIMU();
            telemetry.addLine("Pinpoint IMU Reset!");
        }
        if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) {
            pinpoint.recalibrateIMU();
            telemetry.addLine("Pinpoint IMU Recalibrated!");
        }

        String data = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                currentPose.getX(DistanceUnit.INCH),
                currentPose.getY(DistanceUnit.INCH),
                currentPose.getHeading(AngleUnit.DEGREES)
        );
        telemetry.addData("Position", data);
        telemetry.addData("Heading Lock", isHeadingLocked ? "ON" : "OFF");

        // Add PID constants to telemetry for real-time tuning
        telemetry.addData("--- PID Constants ---", "");
        telemetry.addData("KP", HEADING_KP_TX);
        telemetry.addData("KI", HEADING_KI_TX);
        telemetry.addData("KD", HEADING_KD_TX);
        telemetry.addData("Min Power", ROTATION_MIN_POWER);




        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }

    private void configurePinpoint() {
        pinpoint.setOffsets(2.3 * 25.4, 1 * 25.4, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.resetPosAndIMU();
    }
}
