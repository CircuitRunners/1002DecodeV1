package org.firstinspires.ftc.teamcode.Testers;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MemDevDecode.Config.MecanumDrive;
@Disabled
@Configurable
@TeleOp(name = "Field Centric With April Tags Lock - Pedro Units")
public class TestPedroLLConversions extends OpMode {
//Key - LL is one movement on the coordinate plane = 1 meter vs one movement on pedro plane is 0.5 in
        private MecanumDrive drive;
        private GamepadEx player1;
        private GoBildaPinpointDriver pinpoint;
        private Limelight3A limelight;



        // PID constants
        public static double HEADING_KP_TX = 0.02;
        public static double ROTATION_MIN_POWER = 0.0;

        private final ElapsedTime timer = new ElapsedTime();

        // Conversion constants
        private static final double METERS_TO_INCH = 39.37;
        private static final double PEDRO_UNITS_PER_INCH = 2.0;      // 1 Pedro = 0.5 inch
        private static final double PEDRO_CENTER = 72.0;
        private static final double CENTER_INCHES = PEDRO_CENTER * 0.5;

        // Current pose in Pedro units
        private double robotXP = PEDRO_CENTER;
        private double robotYP = PEDRO_CENTER;
        private double robotHeadingDeg = 0.0;

        // Auto heading lock state
        private boolean autoHeadingLockActive = false;
        private double desiredHeadingDeg = 0.0;
        private static final double HEADING_TOLERANCE_DEG = 2.0;

        @Override
        public void init() {
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
            double strafe = player1.getLeftX();
            double rotate = player1.getRightX();

            pinpoint.update();
            Pose2D currentPose = pinpoint.getPosition();
            limelight.updateRobotOrientation(pinpoint.getPosition().getHeading(AngleUnit.RADIANS));

            double finalRotation;
            LLResult result = limelight.getLatestResult();

            // --- Relocalize from Limelight ---
            if (player1.isDown(GamepadKeys.Button.DPAD_UP)) {
                if (result != null && result.isValid()) {
                    for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                        if (fr.getFiducialId() == 20 || fr.getFiducialId() == 24) {
                            // Limelight reports in meters (origin = field center)
//                            double llX_in = result.getBotpose_MT2().getPosition().x * METERS_TO_INCH;
//                            double llY_in = result.getBotpose_MT2().getPosition().y * METERS_TO_INCH;
//                            //ur gay
//                            double headingRad = Math.toRadians(currentPose.getHeading(AngleUnit.DEGREES));
//
//// Update Pinpoint pose in inches
//                            Pose2D newPose = new Pose2D(DistanceUnit.INCH, llX_in, llY_in,
//                                    AngleUnit.RADIANS, headingRad);
//                            pinpoint.setPosition(newPose);
//
//// Pedro units (2 units per inch)
//                            robotXP = llX_in * PEDRO_UNITS_PER_INCH;
//                            robotYP = llY_in * PEDRO_UNITS_PER_INCH;
//                            robotHeadingDeg = Math.toDegrees(headingRad);
//
//                            telemetry.addData("Relocalized with Tag", fr.getFiducialId());
//                            telemetry.addData("LL meters", "X: %.2f, Y: %.2f", llX_in, llY_in);
//                            telemetry.addData("LL -> Pedro", "X: %.2f, Y: %.2f", robotXP, robotYP);
//                            telemetry.addLine("");
//                            double llX_m = result.getBotpose_MT2().getPosition().x;
//                            double llY_m = result.getBotpose_MT2().getPosition().y;

                           // telemetry.addData("LL raw (m)", "X: %.3f, Y: %.3f", llX_m, llY_m);
                            if (player1.isDown(GamepadKeys.Button.DPAD_UP) && result != null && result.isValid()) {
                                Pose3D mt2Pose = result.getBotpose_MT2();
                                if (mt2Pose != null) {

                                    // --- Step 1: Raw LL pose (meters, origin=center) ---
                                    double llX_m = mt2Pose.getPosition().x;
                                    double llY_m = mt2Pose.getPosition().y;

                                    telemetry.addData("[Step1] LL raw (m)", "X: %.2f, Y: %.2f", llX_m, llY_m);

                                    // --- Step 2: Convert meters → inches ---
                                    double llX_in = llX_m * METERS_TO_INCH;
                                    double llY_in = llY_m * METERS_TO_INCH;

                                    telemetry.addData("[Step2] LL inches (center origin)", "X: %.2f, Y: %.2f", llX_in, llY_in);

                                    // --- Step 3: Shift to bottom-left origin ---
                                    double llX_in_shifted = llX_in + 72.0;
                                    double llY_in_shifted = llY_in + 72.0;

                                    telemetry.addData("[Step3] Shifted inches (field origin)", "X: %.2f, Y: %.2f", llX_in_shifted, llY_in_shifted);

                                    // --- Step 4: Update Pinpoint with X/Y only, keep heading from Pinpoint ---
                                    double currentHeadingRad = currentPose.getHeading(AngleUnit.RADIANS);  // keep heading from Pinpoint
                                    Pose2D newPose = new Pose2D(DistanceUnit.INCH,
                                            llX_in_shifted, llY_in_shifted,
                                            AngleUnit.RADIANS, currentHeadingRad);
                                    pinpoint.setPosition(newPose);

                                    telemetry.addData("[Step4] Pinpoint Pose", "X: %.2f in, Y: %.2f in, H: %.1f°",
                                            llX_in_shifted, llY_in_shifted, Math.toDegrees(currentHeadingRad));



                                    telemetry.addLine("Relocalized MT2 X/Y, kept heading from Pinpoint!");
                                }

                            }


                            break;
                        }
                    }
                }
            }

            // --- Circle pressed: auto heading lock ---
            if (player1.wasJustPressed(GamepadKeys.Button.CIRCLE)) {
                if (result != null && result.isValid()) {
                    for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                        if (fr.getFiducialId() == 24) {
                            desiredHeadingDeg = 45.0;
                            autoHeadingLockActive = true;
                            break;
                        } else if (fr.getFiducialId() == 20) {
                            desiredHeadingDeg = 315.0;
                            autoHeadingLockActive = true;
                            break;
                        }
                    }
                }
            }

            // --- Auto heading lock PID ---
            if (autoHeadingLockActive) {
                double currentHeadingDeg = currentPose.getHeading(AngleUnit.DEGREES);
                double error = desiredHeadingDeg - currentHeadingDeg;

                // Normalize to [-180, 180]
                error = ((error + 180) % 360) - 180;

                double rotationPower = error * HEADING_KP_TX;
                if (Math.abs(rotationPower) > 0 && Math.abs(rotationPower) < ROTATION_MIN_POWER) {
                    rotationPower = Math.signum(rotationPower) * ROTATION_MIN_POWER;
                }

                if (Math.abs(error) < HEADING_TOLERANCE_DEG) {
                    rotationPower = 0.0;
                    autoHeadingLockActive = false;
                }

                finalRotation = rotationPower;
                telemetry.addData("Auto Heading Lock Active", true);
            } else {
                finalRotation = rotate;
                telemetry.addData("Auto Heading Lock Active", false);
            }

            // --- Field-centric drive ---
            double robotHeadingRad = -Math.toRadians(currentPose.getHeading(AngleUnit.DEGREES));
            double theta = Math.atan2(forward, strafe);
            double r = Math.hypot(forward, strafe);
            theta = ((theta - robotHeadingRad + Math.PI) % (2 * Math.PI)) - Math.PI;

            double newForward = r * Math.sin(theta);
            double newStrafe = r * Math.cos(theta);

            drive.drive(newForward, newStrafe, finalRotation);

            // --- Reset buttons ---
            if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) {
                pinpoint.resetPosAndIMU();
                telemetry.addLine("Pinpoint IMU Reset!");
            }
            if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) {
                pinpoint.recalibrateIMU();
                telemetry.addLine("Pinpoint IMU Recalibrated!");
            }

            // --- Sync Pedro from Pinpoint ---
            double pinXInches = currentPose.getX(DistanceUnit.INCH);
            double pinYInches = currentPose.getY(DistanceUnit.INCH);
            double pinHeadingDeg = currentPose.getHeading(AngleUnit.DEGREES);



            // --- Telemetry ---
            telemetry.addData("Position",
                    "{Pinpoint (in) X: %.2f, Y: %.2f, H: %.1f}",
                    pinXInches, pinYInches, pinHeadingDeg);

            telemetry.addData("--- PID ---", "");
            telemetry.addData("KP", HEADING_KP_TX);
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
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.REVERSED
            );
            pinpoint.resetPosAndIMU();
        }
    }


