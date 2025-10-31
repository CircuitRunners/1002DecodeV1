package org.firstinspires.ftc.teamcode.V1.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.lynx.LynxModule;


import java.util.List;

import org.firstinspires.ftc.teamcode.V1.Config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.V1.Config.subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.V1.Config.subsystems.LimelightCamera.BallOrder;
import org.firstinspires.ftc.teamcode.V1.Config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.V1.Config.util.HeadingAutoAligner;
import org.firstinspires.ftc.teamcode.V1.Config.util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Goal Side Auto", preselectTeleOp = "v1Teleop")
public class GoalSideAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer;

    private HeadingAutoAligner aligner;
    private Shooter shooter;
    private Intake intake;
    private LimelightCamera limelight;
    private int pathState;
    private int shotCounter = 0;

    private double lastShooterVelo = 0;
    private boolean shotDetected = false;
    private final double DROP_THRESHOLD = 45;      // RPM drop to count as a shot (tune this)
    private final double RECOVER_THRESHOLD = 20;   // How close to target before re-arming detection
    private final double SHOOTER_READY_THRESHOLD = 0.8; // 80% of target before enabling detection

    private final double shooterDesiredVelo = 1000;
    private Poses.Alliance lastKnownAlliance = null;




    // Tolerance for alignment in radians (approx. 2 degrees)
   // private static final double ALIGN_THRESHOLD = Math.toRadians(2);


    private PathChain travelToShoot, travelToIntake1, intake1, travelBackToShoot1, travelToIntake2, intake2,travelBackToLineup2, travelBackToShoot2, travelToIntake3, intake3, travelBackToShoot3, travelToGate;

    public void buildPaths() {
        // --- Alliance-Aware Path Generation ---
        // Poses.get(POSE_NAME) ensures the correct RED or BLUE coordinates are used.

        // Path 1: Travel from Start to a Shooting Position
//        travelToShoot = follower.pathBuilder()
//                .addPath(new BezierLine(Poses.get(Poses.startPoseGoalSide), Poses.get(Poses.shootPositionGoalSide)))
//                .setLinearHeadingInterpolation(Poses.get(Poses.startPoseGoalSide).getHeading(), Poses.get(Poses.shootPositionGoalSide).getHeading(), 0.8) // endtime is between 0.0 and 1.0
//                .build();

//        travelToShoot = follower.pathBuilder()
//                .addPath(new BezierLine(Poses.get(Poses.startPoseGoalSide), Poses.get(Poses.shootPositionGoalSide)))
//                .setHeadingInterpolation(
//                        HeadingInterpolator.piecewise(
//                                new HeadingInterpolator.PiecewiseNode(
//                                        0,0.4,HeadingInterpolator.linear(Math.toRadians(0), Math.toRadians(45))
//                                ),
//                                new HeadingInterpolator.PiecewiseNode(
//                                        0.4,0.65,HeadingInterpolator.constant(Math.toRadians(45))
//                                ),
//                                new HeadingInterpolator.PiecewiseNode(
//                                        0.65,1,HeadingInterpolator.linear(Math.toRadians(45), Poses.get(Poses.shootPositionGoalSide).getHeading())
//                                ))) // splits up different timed portions of the path (from 0.0-1.0) into different movment types
//                        .build();
        travelToShoot = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.startPoseGoalSide), Poses.get(Poses.shootPositionGoalSide)))
                .setLinearHeadingInterpolation(0, Poses.get(Poses.shootPositionGoalSide).getHeading())

                .build();

        // Path 2: Travel from Shooting Position to Intake Position
        travelToIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.shootPositionGoalSide), Poses.get(Poses.lineupLine1)))
                .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide).getHeading(), Poses.get(Poses.lineupLine1).getHeading())
                .build();
        intake1 = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.lineupLine1), Poses.get(Poses.pickupLine1)))
                .setConstantHeadingInterpolation(Poses.get(Poses.pickupLine1).getHeading())
                .build();

        travelBackToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.pickupLine1), Poses.get(Poses.shootPositionGoalSide2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine1).getHeading(), Poses.get(Poses.shootPositionGoalSide2).getHeading())
                .build();
        travelToIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.shootPositionGoalSide), Poses.get(Poses.lineupLine2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide).getHeading(), Poses.get(Poses.lineupLine2).getHeading())
                .build();
        intake2 = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.lineupLine2), Poses.get(Poses.pickupLine2)))
                .setConstantHeadingInterpolation(Poses.get(Poses.pickupLine2).getHeading())
                .build();
        travelBackToLineup2 = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.pickupLine2), Poses.get(Poses.lineupLine2)))
                .setConstantHeadingInterpolation(Poses.get(Poses.pickupLine2).getHeading())
                .build();
//
//        travelBackToShoot2 = follower.pathBuilder()
//                .addPath(new BezierLine(Poses.get(Poses.lineupLine2), Poses.get(Poses.shootPositionGoalSide)))
//                .setLinearHeadingInterpolation(Poses.get(Poses.lineupLine2).getHeading(), Poses.get(Poses.shootPositionGoalSide).getHeading())
//                .build();
        travelBackToShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.pickupLine2), Poses.get(Poses.backToShoot2ControlPoint), Poses.get(Poses.shootPositionGoalSide2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine2).getHeading(), Poses.get(Poses.shootPositionGoalSide2).getHeading())
                .build();
        travelToIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.shootPositionGoalSide), Poses.get(Poses.lineupLine3)))
                .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide).getHeading(), Poses.get(Poses.lineupLine3).getHeading())
                .build();
        intake3 = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.lineupLine3), Poses.get(Poses.pickupLine3)))
                .setConstantHeadingInterpolation(Poses.get(Poses.pickupLine3).getHeading())
                .build();

        travelBackToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.pickupLine3), Poses.get(Poses.shootPositionGoalSide)))
                .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine3).getHeading(), Poses.get(Poses.shootPositionGoalSide).getHeading())
                .build();
        travelToGate = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.shootPositionGoalSide), Poses.get(Poses.lineupAtGate)))
                .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide).getHeading(), Poses.get(Poses.lineupAtGate).getHeading())
                .build();

        // Initialize aligner to point at a generic goal target (e.g., X=0, Y=144 for blue goal)

        if (Poses.getAlliance() == Poses.Alliance.RED) {
            aligner = new HeadingAutoAligner(12, 132);
        } else {
            aligner = new HeadingAutoAligner(132, 132);
        }
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Initial Travel to Shoot Position
//                intake.intakeRetainBalls();
//                intake.intakeIn();
                if(follower.getCurrentTValue() >=0.4){
                    //limelight.getTagId();

                }
                if (!follower.isBusy()) {
                    follower.followPath(travelToShoot, true);
                    setPathState();
                }
                break;
            case 1: // Shooter Shoot
                if (!follower.isBusy()) {
                    shootBalls();
                }
                break;
            case 2: //go to intake
//                shooter.stopShooter();    // breaks intake
                intake.intakeOut();
//                intake.intakeIdle();
                if (!follower.isBusy()) {

                    follower.followPath(travelToIntake1, true);
                    setPathState();
                }
                break;
            case 3: //intake
                intake.intakeIn();
                setPathState();
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(intake1, true);
                    setPathState();
                }
                break;
            case 5: //go to shoot
//                intake.intakeRetainBalls();
                intake.setServoPower(0);
                if (!follower.isBusy()) {
//                    shootBalls();
                    follower.followPath(travelBackToShoot1, true);

                    setPathState();
                }
                break;
            case 6: //shoot
                if (!follower.isBusy()) {
                    shootBalls();
                }
                break;
            case 7: //go to intake
//                shooter.stopShooter();    // breaks intake
                intake.intakeOut();
                if (!follower.isBusy()) {
                    follower.followPath(travelToIntake2, true);
                    setPathState();
                }
                break;
            case 8: //intake
                intake.intakeIn();
                setPathState();
                break;
            case 9:
                if (!follower.isBusy()) {

                    follower.followPath(intake2, true);

                    setPathState(67);
                }
                break;
            case 10:
//                intake.intakeRetainBalls();
                intake.setServoPower(0);
                if (!follower.isBusy()){
                    follower.followPath(travelBackToLineup2, true);
                    setPathState(8);
                }
                break;
            case 11: //go to shoot
//                intake.intakeRetainBalls();
//                intake.setServoPower(0);
                if (!follower.isBusy()) {
//                    shootBalls();
                    follower.followPath(travelBackToShoot2, true);

                    setPathState();
                }
                break;
            case 12: //shoot
                if (!follower.isBusy()) {
                    shootBalls();
                }
                break;
            case 13:
                setPathState(14);
                break;
//            case 10: //go to intake
//                shooter.stopShooter();
//                intake.intakeOut();
//                if (!follower.isBusy()) {
//                    follower.followPath(travelToIntake3, true);
//                    setPathState(-1);
//                }
//                break;
//            case 11: //intake
//                intake.intakeIn();
//                if (!follower.isBusy()) {
//
//                    follower.followPath(intake3, true);
//
//                    setPathState();
//                }
//                break;
//            case 12: //go to shoot
//
//                if (!follower.isBusy()) {
//                    intake.intakeRetainBalls();
//                    follower.followPath(travelBackToShoot3, true);
//
//                    setPathState();
//                }
//                break;
//            case 13: //shoot
//                if (follower.isBusy()) {
//                    shootBalls();
//                }
//                break;
            case 14:
                intake.intakeIdle();
                shooter.stopShooter();
                intake.setServoPower(0);
                if (!follower.isBusy()){
                    follower.followPath(travelToGate, true);
                    setPathState(-1);
                }


            default: // End State (-1)
                shooter.stopShooter();
                intake.intakeIdle();
                intake.setServoPower(0);
                if (!follower.isBusy()) {
                    requestOpModeStop();
                }
        }
    }




    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void setPathState() {
        pathState += 1;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        telemetry.addData("X Pos", follower.getPose().getX());
        telemetry.addData("Y Pos", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path State", pathState);
        telemetry.addData("Shots Fired", shotCounter);
        telemetry.addData("Shooter Velocity", shooter.getCurrentVelo());
        telemetry.addData("Intake Power", intake.getPower());

       // telemetry.addData("Shooter Velo: ", shooter.getCurrentVelo());

        telemetry.update();




        autonomousPathUpdate();

    }

    @Override
    public void stop() {
        shooter.stopShooter();
        intake.intakeIdle();
        intake.setServoPower(0);
        Poses.savePose(follower.getPose());
        //limelight.limelightCamera.pause();
        //shooterIntake.stopAll();
    }

    @Override
    public void init() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        pathTimer = new Timer();


        follower = Constants.createFollower(hardwareMap);


        // No pathing or follower updates here, just initialize

        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
//        limelight = new LimelightCamera(hardwareMap);
//        limelight.limelightCamera.start();
//
//        BallOrder detected = limelight.detectBallOrder();
//
//        telemetry.addData("Ball Order", detected);
        telemetry.update();
    }

    @Override
    public void init_loop() {


        Poses.updateAlliance(gamepad1, telemetry);


        if (Poses.getAlliance() != lastKnownAlliance) {

            follower.setStartingPose(Poses.get(Poses.startPoseGoalSide));


            buildPaths();

            lastKnownAlliance = Poses.getAlliance();
            telemetry.addData("STATUS", "Paths Rebuilt for " + lastKnownAlliance);
        }


        telemetry.addData("Alliance Set", Poses.getAlliance());
        telemetry.addData("Start Pose", Poses.get(Poses.startPoseGoalSide));
        telemetry.update();
    }

    @Override
    public void start() {
        //buildPaths();
        pathTimer.resetTimer();
        setPathState(0);
    }

    public void shootBalls(){
    ////////////
        /*ONLY USING PATH TIMER STUFF - NOT CURRENTLY USING SHOOTING TIMER OR THE DELTA STUFF*/
    ////////////
        shooter.shoot(shooterDesiredVelo);
        double currentVelo = shooter.getCurrentVelo();
        boolean shootingStarted = false;
        Timer shootingTimer = new Timer();

        // --- Skip detection until shooter is near speed ---
        if (currentVelo < shooterDesiredVelo * SHOOTER_READY_THRESHOLD) {
            lastShooterVelo = currentVelo;
            intake.intakeIn();
            intake.setServoPower(0);
        }

        // --- Feed balls only when shooter is up to speed ---
        if (currentVelo >= shooterDesiredVelo - 25 && currentVelo <= shooterDesiredVelo + 55) {
            intake.intakeIn();
            intake.setServoPower(1);
            shootingStarted = true;

        } else {
            intake.intakeRetainBalls();
            intake.setServoPower(0);
        }

        if (shootingStarted){
            shootingTimer.resetTimer();
        }

        // --- Detect velocity drop (shot fired) ---
        double delta = lastShooterVelo - currentVelo;


        // If speed dropped sharply, count one shot
        if (!shotDetected && delta > DROP_THRESHOLD) {
            shotDetected = true;
            shotCounter++;
            telemetry.addData("Shot Detected", shotCounter);
        }

        // When shooter recovers, re-arm detection for next ball
        if (shotDetected && currentVelo >= shooterDesiredVelo - RECOVER_THRESHOLD) {
            shotDetected = false;
        }

        // --- Move to next path after 3 confirmed shots ---
        //og was >= 7.7
        if (pathTimer.getElapsedTimeSeconds() >= 7) {
            shotCounter = 0;
            setPathState();
        }

        lastShooterVelo = currentVelo;
    }


    /*
    LESS CHOPPED SHOOT BALLS HERE:
    public void shootBalls(){
        // Start the shooter flywheel immediately.
        shooter.shoot(shooterDesiredVelo);
        double currentVelo = shooter.getCurrentVelo();

        // Assume shooting has NOT started until we meet the velocity target.
        boolean isShooterReady = currentVelo >= shooterDesiredVelo - 25 && currentVelo <= shooterDesiredVelo + 55;

        if (isShooterReady) {
            // When ready, feed the ball using both the main intake motor and the servo.
            // This is the only place we command intake/servo ON.
            intake.intakeIn();
            intake.setServoPower(1);
        } else {
            // When NOT ready, or if speed drops too far, stop feeding.
            // This is the only place we command intake/servo OFF/retain.
            // Use intakeIdle() or intakeRetainBalls() depending on what state you want
            // the main intake motor to be in when waiting. We'll use intakeIdle()
            // to ensure the motor isn't fighting itself.
            intake.intakeIdle();
            intake.setServoPower(0);
        }

        // --- Shot Detection Logic (Remains the same) ---
        // ... your shot detection and path state transition logic here ...

        // --- Detect velocity drop (shot fired) ---
        double delta = lastShooterVelo - currentVelo;

        // If speed dropped sharply, count one shot
        if (!shotDetected && delta > DROP_THRESHOLD) {
            shotDetected = true;
            shotCounter++;
            telemetry.addData("Shot Detected", shotCounter);
        }

        // When shooter recovers, re-arm detection for next ball
        if (shotDetected && currentVelo >= shooterDesiredVelo - RECOVER_THRESHOLD) {
            shotDetected = false;
        }

        // --- Move to next path after timeout ---
        if (pathTimer.getElapsedTimeSeconds() >= 7) {
            shotCounter = 0;
            setPathState();
        }

        lastShooterVelo = currentVelo;
    }

     */

}