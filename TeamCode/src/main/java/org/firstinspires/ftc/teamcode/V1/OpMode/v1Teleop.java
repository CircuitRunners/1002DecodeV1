package org.firstinspires.ftc.teamcode.V1.OpMode;

import com.bylazar.configurables.annotations.Configurable;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
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
import org.firstinspires.ftc.teamcode.V1.Config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.V1.Config.subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.V1.Config.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.V1.Config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.V1.Config.util.Poses;
import org.firstinspires.ftc.teamcode.V1.Config.util.ValidShootingZoneChecker;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.Locale;
import java.util.function.Supplier;

@TeleOp (name = "Quatro Teleop", group = "a")
@Configurable
public class v1Teleop extends OpMode {

    private MecanumDrive drive;
    private GoBildaPinpointDriver pinpoint;
    private Follower follower;
    private LimelightCamera limelight;
    private ValidShootingZoneChecker zoneChecker;
    private Intake intake;
    private Shooter shooter;
    private Supplier<PathChain> pathChainBlue;
    private Supplier<PathChain> pathChainRed;

    private GamepadEx player1;
    TriggerReader leftTriggerReader;
    TriggerReader rightTriggerReader;

    private final ElapsedTime timer = new ElapsedTime();

    private int stateMachine = -1;
    private static final double METERS_TO_INCH = 39.37;

    // State variable to manage the heading lock.
    private boolean isHeadingLocked = false;
    private boolean isRedAlliance;
    private boolean automatedDrive = false;
    private boolean preselectFromAuto = false;
    private boolean isIntakeInUse = false;
    private final double MINIMUM_SHOOTER_VELO = 1600; //   tick/sec
    private final double MAXIMUM_SHOOTER_VELO = 1650; //   tick/sec



    @Override
    public void init() {
        telemetry.addLine("Initializing...");
        telemetry.update();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Poses.getStartingPose());
        follower.update();

        player1 = new GamepadEx(gamepad1);


        leftTriggerReader = new TriggerReader(
                player1, GamepadKeys.Trigger.LEFT_TRIGGER
        );
        rightTriggerReader = new TriggerReader(
                player1, GamepadKeys.Trigger.RIGHT_TRIGGER
        );

        drive = new MecanumDrive();
        drive.init(hardwareMap);

        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configurePinpoint();

        Pose2D newPose = new Pose2D(DistanceUnit.INCH, Poses.getStartingPose().getX(), Poses.getStartingPose().getY(), AngleUnit.RADIANS, Math.toRadians(Poses.getStartingPose().getHeading()));
        pinpoint.setPosition(newPose);


        limelight = new LimelightCamera(hardwareMap);
        limelight.limelightCamera.pipelineSwitch(3);
        limelight.limelightCamera.start();

        zoneChecker = new ValidShootingZoneChecker();

        // Set up bulk caching
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        if(Poses.getAlliance() !=null){
            if(Poses.getAlliance() == Poses.Alliance.RED){
                isRedAlliance = true;
                preselectFromAuto = true;
            }
            else {
                isRedAlliance = false;
                preselectFromAuto = true;
            }
        }
        else{
            isRedAlliance = false;
            preselectFromAuto = false;
        }


            pathChainBlue = () -> follower.pathBuilder() //Lazy Curve Generation for blue shoot pos
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(44, 99))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(135), 0.8))
                .build();


            pathChainRed = () -> follower.pathBuilder() //Lazy Curve Generation for red shoot pos
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(100, 99))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();






        telemetry.addLine("Ready!");
        telemetry.update();
    }

    @Override
    public void loop() {
        //boolean control to check if pedro is in use
        if (follower.isBusy()){
            automatedDrive = true;
        }
        //gamepad logic
        player1.readButtons();
        leftTriggerReader.readValue();
        boolean leftBumperJustPressed = player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER);
        boolean leftTriggerJustPressed = leftTriggerReader.wasJustPressed();
        boolean rightBumperJustPressed = player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER);
        boolean rightTriggerJustPressed = rightTriggerReader.wasJustPressed();

        //localization + pedropathing logic
        pinpoint.update();
        Pose2D currentPose = pinpoint.getPosition();
        follower.update();

        if (player1.wasJustPressed(GamepadKeys.Button.TOUCHPAD_FINGER_1)){
            updateCoordinates();
        }

        //selector logic for alliance
        if (!preselectFromAuto) {
            if (player1.wasJustPressed(GamepadKeys.Button.START)) {
                isRedAlliance = true;
            } else if (player1.wasJustPressed(GamepadKeys.Button.BACK)) {
                isRedAlliance = false;
            }
        }

        //state machine control
        if (rightBumperJustPressed) {
                stateMachine = (stateMachine + 1) % 2;
        }

        if (leftBumperJustPressed) {
            stateMachine = (stateMachine - 1) % 2;

        }

        //drive logic
        double forward = player1.getLeftY();
        double strafe = player1.getLeftX();
        double rotate = player1.getRightX();

        double robotHeading = Math.toRadians(currentPose.getHeading(AngleUnit.DEGREES));
        double finalRotation;

        if (isHeadingLocked) {
            finalRotation = limelight.autoAlign();
            telemetry.addData("Using Limelight Data for Heading Lock.", "");
            telemetry.addData("Error from tag center", limelight.error);
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

        if(!follower.isBusy()) {
            drive.drive(newForward, newStrafe, finalRotation);
        }


        // recalibrate pinpoint
//        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) {
//            pinpoint.resetPosAndIMU();
//            telemetry.addLine("Pinpoint IMU Reset!");
//        }
//        if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) {
//            pinpoint.recalibrateIMU();
//            telemetry.addLine("Pinpoint IMU Recalibrated!");
//        }
        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) {
            pinpoint.recalibrateIMU();
            Pose2D newPose = new Pose2D(DistanceUnit.INCH,
                    72,72,
                    AngleUnit.RADIANS, 0);
            pinpoint.setPosition(newPose);
            telemetry.addLine("Pinpoint Reset - Position now 72,72 (Middle)!");
        }

        String data = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                currentPose.getX(DistanceUnit.INCH),
                currentPose.getY(DistanceUnit.INCH),
                currentPose.getHeading(AngleUnit.DEGREES)
        );

        //main state machine loop where all of the actions actually happen
        switch (stateMachine){
            case 0:
                shooter.stopShooter();
               //intake stuff
                 isIntakeInUse= true;
                if(rightTriggerJustPressed){
                    intake.intakeIn();
                    intake.setServoPower(0.45);
                }
                else if (leftTriggerJustPressed){
                    intake.intakeOut();
                    intake.setServoPower(-0.45);
                }
                else{
                    intake.intakeIdle();
                }

                break;
            case 1:
            //score stuff

                if (!isIntakeInUse){
                    intake.intakeRetainBalls();
                }
                if (player1.wasJustPressed(GamepadKeys.Button.CIRCLE)) {
                    if (isRedAlliance){
                        follower.followPath(pathChainRed.get());
                    }
                    else {
                        follower.followPath(pathChainBlue.get());
                    }


                }
                if (!automatedDrive){
                    //isHeadingLocked = true;
                }
                else {
                    isHeadingLocked = false;
                }


               // if (zoneChecker.isInsideShootingZone(currentPose.getX(DistanceUnit.INCH), currentPose.getY(DistanceUnit.INCH))) {
                    shooter.shoot();
                    //shooting logic
                    if ((shooter.getCurrentVelo() >= MINIMUM_SHOOTER_VELO) && (shooter.getCurrentVelo() <= MAXIMUM_SHOOTER_VELO)) {
                        isIntakeInUse = true;
                        intake.intakeIn();
                        if (player1.isDown(GamepadKeys.Button.CROSS)) {
                            intake.setServoPower(1);
                        }
                        else{
                            intake.setServoPower(0);
                        }
                    }
                    else {
                        isIntakeInUse = false;
                    }
               // }
//                else {
//                        shooter.stopShooter();
//                        isIntakeInUse = false;
//                }
                break;
            default:
                isHeadingLocked = false;
                break;
        }
        if (stateMachine > 1){
            stateMachine = -1;
        }
        telemetry.addData("Position", data);
        telemetry.addData("State", stateMachine);
        telemetry.addData("Heading Lock", isHeadingLocked ? "ON" : "OFF");
        telemetry.addData("Alliance: ", isRedAlliance? "Red" : "Blue");
        telemetry.update();


    }

    @Override
    public void stop() {
        limelight.limelightCamera.stop();
    }

    //takes position from april tag and updates pinpoint
    public void updateCoordinates() {
        //updates the orientation of robot for limelight camera's usage
        limelight.limelightCamera.updateRobotOrientation(pinpoint.getPosition().getHeading(AngleUnit.RADIANS));

        LLResult result = limelight.getresult();

        //ensures result exsists and is from an acceptable apriltag
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                if (fr.getFiducialId() == 20 || fr.getFiducialId() == 24) {
                    Pose3D mt2Pose = result.getBotpose_MT2();
                    if (mt2Pose != null) {
                        //gets raw position from limelight in m
                        double llX_m = mt2Pose.getPosition().x;
                        double llY_m = mt2Pose.getPosition().y;
                        //converts position to inches
                        double llX_in = llX_m * METERS_TO_INCH;
                        double llY_in = llY_m * METERS_TO_INCH;
                        //shifts position to bottom left corner field origin for pedro pathing use
                        double llX_in_shifted = llX_in + 72.0;
                        double llY_in_shifted = llY_in + 72.0;

                        double currentHeadingRad = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);  // keep heading from Pinpoint
                        Pose2D newPose = new Pose2D(DistanceUnit.INCH,
                                llX_in_shifted, llY_in_shifted,
                                AngleUnit.RADIANS, currentHeadingRad);
                        pinpoint.setPosition(newPose);
                        telemetry.addData("New Pose From Apriltag:", "X: %.2f in, Y: %.2f in, H: %.1f°", llX_in_shifted, llY_in_shifted, Math.toDegrees(currentHeadingRad));
                    }
                    break;
                }
            }
        }


    }

    //define pinpoint offsets and constants
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