package org.firstinspires.ftc.teamcode.V1.OpMode;

import com.bylazar.configurables.annotations.Configurable;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.gamepad.TriggerReader;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.V1.Config.subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.V1.Config.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.V1.Config.util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.Locale;

@TeleOp (name = "Quatro Teleop", group = "a")
@Configurable
public class v1Teleop extends OpMode {

    private MecanumDrive drive;
    TriggerReader leftTriggerReader;
    //private Follower follower;

    private LimelightCamera limelight;
    private GamepadEx player1;
    private GoBildaPinpointDriver pinpoint;
    private final ElapsedTime timer = new ElapsedTime();

    // State variable to manage the heading lock.
    private boolean isHeadingLocked = false;

    private int stateMachine = -1;

    private static final double METERS_TO_INCH = 39.37;


    @Override
    public void init() {
        telemetry.addLine("Initializing...");
        telemetry.update();

//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(Poses.getStartingPose());
//        follower.update();


      //  Pose2D newPose = new Pose2D(DistanceUnit.MM, Poses.getStartingPose().getX(), Poses.getStartingPose().getY(), AngleUnit.RADIANS, Math.toRadians(Poses.getStartingPose().getHeading()));
      //  pinpoint.setPosition(newPose);
        player1 = new GamepadEx(gamepad1);

        leftTriggerReader = new TriggerReader(
                player1, GamepadKeys.Trigger.LEFT_TRIGGER
        );

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
        leftTriggerReader.readValue();
        pinpoint.update();
        Pose2D currentPose = pinpoint.getPosition();
        updateCoordinates();
        //follower.update();


        boolean leftBumperJustPressed = player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER);
        boolean leftTriggerJustPressed = leftTriggerReader.wasJustPressed();


        if (leftBumperJustPressed) {
                stateMachine = (stateMachine + 1) % 2;
        }

        if (leftTriggerJustPressed) {
            stateMachine = (stateMachine - 1) % 2;

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

    public void updateCoordinates() {
        LLResult result = limelight.getresult();
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                if (fr.getFiducialId() == 20 || fr.getFiducialId() == 24) {
                    Pose3D mt2Pose = result.getBotpose_MT2();
                    if (mt2Pose != null) {

                        double llX_m = mt2Pose.getPosition().x;
                        double llY_m = mt2Pose.getPosition().y;

                        double llX_in = llX_m * METERS_TO_INCH;
                        double llY_in = llY_m * METERS_TO_INCH;

                        double llX_in_shifted = llX_in + 72.0;
                        double llY_in_shifted = llY_in + 72.0;

                        double currentHeadingRad = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);  // keep heading from Pinpoint
                        Pose2D newPose = new Pose2D(DistanceUnit.INCH,
                                llX_in_shifted, llY_in_shifted,
                                AngleUnit.RADIANS, currentHeadingRad);
                        pinpoint.setPosition(newPose);
                        telemetry.addData("Pinpoint Pose", "X: %.2f in, Y: %.2f in, H: %.1f°", llX_in_shifted, llY_in_shifted, Math.toDegrees(currentHeadingRad));
                    }
                    break;
                }
            }
        }


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
