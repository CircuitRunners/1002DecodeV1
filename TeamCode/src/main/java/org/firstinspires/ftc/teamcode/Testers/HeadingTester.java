package org.firstinspires.ftc.teamcode.Testers;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.gamepad.TriggerReader;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.V1.Config.subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.V1.Config.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.V1.Config.util.HeadingAutoAligner;

@TeleOp(name = "Heading Auto Aligner (Dashboard Tunable PID)", group = "Testers")
@Configurable
public class HeadingTester extends OpMode {





        // === Tunable PID values (live via FTC Dashboard) ===
        public static double kP = 0.6;
        public static double kI = 0.0;
        public static double kD = 0.02;

        // Fixed target coordinates (can also be made tunable)
        public static double targetX = 11;
        public static double targetY = 135;

        private GoBildaPinpointDriver pinpoint;
        private MecanumDrive drive;
        private LimelightCamera limelight;
        private GamepadEx player1;
        private TriggerReader leftTriggerReader;
        private TESTIING_HeadingAligner aligner;

        private boolean headingLock = false;
        private boolean useCoordinateAlign = true;

        @Override
        public void init() {
            telemetry.addLine("Initializing...");
            telemetry.update();

            drive = new MecanumDrive();
            drive.init(hardwareMap);

            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
            configurePinpoint();

            limelight = new LimelightCamera(hardwareMap);
            limelight.limelightCamera.pipelineSwitch(3);
            limelight.limelightCamera.start();

            player1 = new GamepadEx(gamepad1);
            leftTriggerReader = new TriggerReader(player1, GamepadKeys.Trigger.LEFT_TRIGGER);

            aligner = new TESTIING_HeadingAligner(targetX, targetY);

            for (LynxModule hub : hardwareMap.getAll(LynxModule.class))
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

            telemetry.addLine("Ready for PID tuning via Dashboard!");
            telemetry.update();
        }

        @Override
        public void loop() {
            player1.readButtons();
            leftTriggerReader.readValue();
            pinpoint.update();

            Pose2D pose = pinpoint.getPosition();
            double robotX = pose.getX(DistanceUnit.INCH);
            double robotY = pose.getY(DistanceUnit.INCH);
            double robotHeading = Math.toRadians(pose.getHeading(AngleUnit.DEGREES));

            // === Mode Toggles ===
            if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                headingLock = !headingLock;
                aligner.reset();
            }
            if (player1.wasJustPressed(GamepadKeys.Button.Y)) {
                useCoordinateAlign = !useCoordinateAlign;
            }

            // === Drive logic ===
            double forward = player1.getLeftY();
            double strafe = player1.getLeftX();
            double rotateInput = player1.getRightX();
            double rotationCommand;

            if (headingLock) {
                if (useCoordinateAlign) {
                    aligner.setTarget(targetX, targetY); // Update if dashboard changes
                    rotationCommand = aligner.getRotationPower(robotX, robotY, robotHeading, kP, kI, kD);
                    telemetry.addLine("Mode: Coordinate AutoAlign");
                } else {
                    rotationCommand = limelight.autoAlign();
                    telemetry.addLine("Mode: Limelight AutoAlign");
                }
            } else {
                rotationCommand = rotateInput;
            }

            // === Field-centric transform ===
            double theta = Math.atan2(forward, strafe);
            double r = Math.hypot(forward, strafe);
            theta = AngleUnit.normalizeRadians(theta - robotHeading);
            double newForward = r * Math.sin(theta);
            double newStrafe = r * Math.cos(theta);

            drive.drive(newForward, newStrafe, rotationCommand);

            telemetry.addData("Pos", "(%.1f, %.1f)", robotX, robotY);
            telemetry.addData("Heading", "%.1f°", pose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Heading Lock", headingLock ? "ON" : "OFF");
            telemetry.addData("Align Mode", useCoordinateAlign ? "Coordinate" : "Limelight");
            telemetry.addLine(String.format("kP: %.3f  kI: %.3f  kD: %.3f", kP, kI, kD));
            telemetry.update();
        }

        @Override
        public void stop() {
            limelight.limelightCamera.stop();
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

