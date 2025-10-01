package org.firstinspires.ftc.teamcode.V1.OpMode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.V1.Config.subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.V1.Config.subsystems.MecanumDrive;

import java.util.List;
import java.util.Locale;

@TeleOp (name = "Quatro Teleop", group = "a")
@Configurable
public class v1Teleop extends OpMode {

    private MecanumDrive drive;

    private LimelightCamera limelight;
    private GamepadEx player1;
    private GoBildaPinpointDriver pinpoint;
    private final ElapsedTime timer = new ElapsedTime();

    // State variable to manage the heading lock.
    private boolean isHeadingLocked = false;

    private int stateMachine = -1;


    @Override
    public void init() {
        telemetry.addLine("Initializing...");
        telemetry.update();

        player1 = new GamepadEx(gamepad1);

        drive = new MecanumDrive();
        drive.init(hardwareMap);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configurePinpoint();

        limelight = new LimelightCamera(hardwareMap);
        limelight.limelightCamera.pipelineSwitch(3);
        limelight.limelightCamera.start();

        // Set up bulk caching
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry.addLine("Ready!");
        telemetry.update();
    }

    @Override
    public void loop() {

        player1.readButtons();

        pinpoint.update();
        Pose2D currentPose = pinpoint.getPosition();


        boolean leftBumperJustPressed = player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER);



        if (leftBumperJustPressed) {
                stateMachine = (stateMachine + 1) % 2;
        }

        double forward = player1.getLeftY();
        double strafe = player1.getLeftX();
        double rotate = player1.getRightX();


        // Toggle the heading lock with dpad up/down
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            isHeadingLocked = true;
            telemetry.addLine("Heading Lock Enabled.");
        }
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            isHeadingLocked = false;
            telemetry.addLine("Heading Lock Disabled.");
        }


        //drive logic
        double robotHeading = Math.toRadians(currentPose.getHeading(AngleUnit.DEGREES));
        double finalRotation;

        if (isHeadingLocked) {
            finalRotation = limelight.autoAlign();

            telemetry.addData("Using Limelight Data for Heading Lock.", "");
            telemetry.addData("PID Error", limelight.error);
            telemetry.addData("PID Rotation", limelight.autoAlign());

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



        // recalibrate pinpoint
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

        switch (stateMachine){
            case 0:


                break;
            case 1:


                break;
            default:
                isHeadingLocked = false;
                break;
        }
        telemetry.addData("Position", data);
        telemetry.addData("Heading Lock", isHeadingLocked ? "ON" : "OFF");
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
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.resetPosAndIMU();
    }


}
