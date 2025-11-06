package org.firstinspires.ftc.teamcode.V1.OpMode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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

@TeleOp (name = "NEW - Quatro Teleop 1.1", group = "a")
@Configurable
public class v1_1Teleop extends OpMode {

    private MecanumDrive drive;
    private GoBildaPinpointDriver pinpoint;
    private Follower follower;
    private LimelightCamera limelight;
    private Intake intake;
    private Shooter shooter;
    private TelemetryManager panelsTelemetry;

    private GamepadEx player1;
    TriggerReader leftTriggerReader;
    TriggerReader rightTriggerReader;
    private int stateMachine = 0;
    private static final double METERS_TO_INCH = 39.37;

    // State variable to manage the heading lock.
    private boolean isHeadingLocked = false;
    private boolean isRedAlliance;
    private boolean automatedDrive = false;
    private boolean preselectFromAuto = false;
    private boolean isIntakeInUse = false;
    private int intakeState = 1; //1 is in, 2 is out, 3 is stop

    boolean isShooterReady = false;
    boolean ball_was_present = true;
    int shotCounter = 0;

    private double desiredVeloRed = 0;
    private double desiredVeloBlue = 0;
    private double shooterIncrement = 0;





    @Override
    public void init() {
        telemetry.addLine("Initializing...");
        telemetry.update();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
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


        limelight = new LimelightCamera(hardwareMap);
        limelight.limelightCamera.start();


        // Set up bulk caching
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
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

        telemetry.addLine("Ready!");
        telemetry.update();
    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.clearBulkCache();
        }
        //boolean control to check if pedro is in use
        if (follower.isBusy()){
            automatedDrive = true;
        }
        else if (!follower.isBusy()){
            automatedDrive = false;
        }

        pinpoint.update();
        Pose2D currentPose = pinpoint.getPosition();
        follower.update();

        // --- CONSOLIDATED SENSOR AND INPUT READS (Added for single-pass data) ---
        double limelightRotation = limelight.autoAlign();
        double shooterVelo = shooter.getCurrentVelo();
        double leftTriggerValue = gamepad1.left_trigger;
        double rightTriggerValue = gamepad1.right_trigger;
        // ------------------------------------------------------------------------


        //gamepad logic
        player1.readButtons();
        leftTriggerReader.readValue();
        boolean leftBumperJustPressed = player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER);
        boolean leftTriggerJustPressed = leftTriggerReader.wasJustPressed();
        boolean rightBumperJustPressed = player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER);
        boolean rightTriggerJustPressed = rightTriggerReader.wasJustPressed();

        boolean leftTriggerIsPressed = leftTriggerReader.isDown();
        boolean rightTriggerIsPressed = rightTriggerReader.isDown();

        //selector logic for alliance

        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
            isRedAlliance = true;
        } else if (player1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            isRedAlliance = false;
        }

        //Relocalize from AprilTags
        if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)){
            updateCoordinates();
        }

        //state machine control
        if (rightBumperJustPressed) {
            stateMachine = (stateMachine + 1) % 3;
        }

        if (leftBumperJustPressed) {
            stateMachine = (stateMachine - 1) % 3;

        }

        //drive logic
        double forward = player1.getLeftY();
        double strafe = player1.getLeftX();
        double rotate = player1.getRightX();

        double robotHeading = Math.toRadians(currentPose.getHeading(AngleUnit.DEGREES));
        double finalRotation = rotate;

        if (isHeadingLocked) {
                if (limelight.getResult().isValid()) {
                    for (LLResultTypes.FiducialResult fr : limelight.getResult().getFiducialResults()) {
                        if (fr.getFiducialId() == 20 || fr.getFiducialId() == 24) {

                            finalRotation = limelightRotation;

                            telemetry.addLine("Using Limelight Data for Heading Lock.");
                            telemetry.addData("Error from tag center", limelight.error);
                            telemetry.addData("PID Rotation", limelightRotation);
                        }
                    }
                }
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
            if (isRedAlliance) {
                drive.drive(-newForward, -newStrafe, finalRotation);
            }
            else{
                drive.drive(newForward, newStrafe, finalRotation);
            }
        }


        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) {
            follower.setPose(new Pose(72,72, Math.toRadians(-90)));
            telemetry.addLine("Pinpoint Reset - Position now 72,72 (Middle)!");
        }


        String followerData = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading())

        );

        //main state machine loop where all of the actions actually happen
        switch (stateMachine){
            case 0:
                isHeadingLocked = false;
                shooter.stopShooter();

                //intake stuff
                if(rightTriggerJustPressed){
                    if (intakeState == 1){
                       intakeState =2;
                    }
                    else if (intakeState == 2){
                        intakeState = 1;
                    }
                    else if (intakeState == 3){
                        intakeState = 1;
                    }
                }
                if (leftTriggerValue > 0.25){
                    intakeState = 3;
                }

                if (intakeState == 1){
                    intake.intakeInDistance();
                    isIntakeInUse= true;
                } else if (intakeState == 2) {
                    intake.fullIntakeOut();
                    isIntakeInUse= true;
                }
                else if (intakeState == 3){
                    intake.fullIntakeIdle();
                    isIntakeInUse= false;
                }

                break;
            case 1:
                //score stuff

                    isIntakeInUse = true;
                    intake.intakeRetainBalls();
                    intake.intakeServoIdle();

                if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) {
                    if (isHeadingLocked){
                        isHeadingLocked = false;
                    }
                    else if (!isHeadingLocked){
                        isHeadingLocked = true;
                    }
                }

                if(isRedAlliance) {
                    desiredVeloRed = shooter.calculateFlywheelVelocity(limelight.calculateDistanceToGoal(follower.getPose().getX(), follower.getPose().getY(), 132, 137)) + shooterIncrement;
                }
                else if (!isRedAlliance){
                    desiredVeloBlue = shooter.calculateFlywheelVelocity(limelight.calculateDistanceToGoal(follower.getPose().getX(), follower.getPose().getY(), 12, 137)) + shooterIncrement;
                }


                if (isRedAlliance){
                    shooter.shoot(desiredVeloRed);
                }
                else if (!isRedAlliance){
                    shooter.shoot(desiredVeloBlue);
                }


                if (gamepad1.dpad_up){
                shooterIncrement +=50;
                } else if (gamepad1.dpad_down) {
                    shooterIncrement -=50;
                }
                else if (gamepad1.dpad_left){
                    shooterIncrement =0;
                }
                else if (gamepad1.dpad_right){
                    shooterIncrement =0;
                }

                break;
            case 2:
                if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) {
                    if (isHeadingLocked){
                        isHeadingLocked = false;
                    }
                    else if (!isHeadingLocked){
                        isHeadingLocked = true;
                    }
                }

                if (gamepad1.dpad_up){
                    shooterIncrement +=50;
                } else if (gamepad1.dpad_down) {
                    shooterIncrement -=50;
                }
                else if (gamepad1.dpad_left){
                    shooterIncrement =0;
                }
                else if (gamepad1.dpad_right){
                    shooterIncrement =0;
                }

                //shooting logic
               shootBalls();


                break;
            default:
                isHeadingLocked = false;
                intakeState = 3;
                stateMachine = 0;
                break;
        }
        if (stateMachine > 2 || stateMachine < 0){
            stateMachine = 0;
        }

        telemetry.addData("FOLLOWER (Pedro) Position", followerData);
        telemetry.addData("State", stateMachine);
        telemetry.addData("Heading Lock", isHeadingLocked ? "ON" : "OFF");
        telemetry.addData("Alliance: ", isRedAlliance? "Red" : "Blue");
        telemetry.addData("Shooter Velo", shooterVelo); // Replaced shooter.getCurrentVelo()
        telemetry.addData("Intake in use: ", isIntakeInUse? "Yes" : "No");


//        panelsTelemetry.debug("Position", followerData);
//        panelsTelemetry.debug("State", stateMachine);
//        panelsTelemetry.debug("Heading Lock", isHeadingLocked ? "ON" : "OFF");
//        panelsTelemetry.debug("Alliance: ", isRedAlliance? "Red" : "Blue");
//        panelsTelemetry.debug("Shooter Velo", shooterVelo); // Replaced shooter.getCurrentVelo()
//        panelsTelemetry.debug("Intake in use: ", isIntakeInUse? "Yes" : "No");




        panelsTelemetry.update(telemetry);
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
        //limelight.limelightCamera.updateRobotOrientation(follower.getHeading());
        limelight.limelightCamera.pipelineSwitch(3);
        LLResult result = limelight.getResult();

        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                if (fr.getFiducialId() == 20 || fr.getFiducialId() == 24) {
                    Pose3D mt1Pose = result.getBotpose();
                    if (mt1Pose != null) {
                        //gets raw position from limelight in m
                        double llX_m = mt1Pose.getPosition().x;
                        double llY_m = mt1Pose.getPosition().y;
                        //converts position to inches
                        double llX_in = llX_m * METERS_TO_INCH;
                        double llY_in = llY_m * METERS_TO_INCH;
                        //rotates limelight points 90 degrees counterclockwise
                        double llX_in_rotated = llY_in;
                        double llY_in_rotated = -llX_in;
                        //shifts position to bottom left corner field origin for pedro pathing use
                        double llX_in_shifted = llX_in_rotated + 72.0;
                        double llY_in_shifted = llY_in_rotated + 72.0;

                        double currentHeadingRad = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);  // keep heading from Pinpoint
                        follower.setPose(new Pose(llX_in_shifted,llY_in_shifted,currentHeadingRad));
                        gamepad1.rumble(500);
                        telemetry.addData("New Pose From Apriltag:", "X: %.2f in, Y: %.2f in, H: %.1f°", llX_in_shifted, llY_in_shifted, Math.toDegrees(currentHeadingRad));
                    }
                    break;
                }
            }
        }


    }

    //define pinpoint offsets and constants
    private void configurePinpoint() {
        pinpoint.setOffsets(1.91, -2.64, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.recalibrateIMU();
    }

    public void shootBalls(){
        double currentVelo = shooter.getCurrentVelo();


        if (isRedAlliance) {
            isShooterReady = currentVelo >= (desiredVeloRed+shooterIncrement) - 30 && currentVelo <= (desiredVeloRed+shooterIncrement) + 45;
        }
        else if (!isRedAlliance){
            isShooterReady = currentVelo >= (desiredVeloBlue+shooterIncrement) - 30 && currentVelo <= (desiredVeloBlue+shooterIncrement) + 45;
        }


        if (isShooterReady) {
            isIntakeInUse = true;
            intake.intakeIn();
            intake.setServoPower(1);
            telemetry.addLine("Shooting");
        }
        else if (!isShooterReady){
            intake.intakeRetainBalls();
            intake.setServoPower(0);
        }

        boolean ball_is_present = (intake.getDistanceMM() <= 60);

        if (ball_was_present && !ball_is_present) {
            shotCounter++;
        }

        ball_was_present = ball_is_present;

        if (shotCounter >= 3) {
            shotCounter = 0;
            stateMachine++;
        }
        telemetry.update();
    }

}