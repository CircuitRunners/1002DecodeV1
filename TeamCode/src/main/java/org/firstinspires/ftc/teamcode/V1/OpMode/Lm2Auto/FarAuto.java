package org.firstinspires.ftc.teamcode.V1.OpMode.Lm2Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.V1.Config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.V1.Config.subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.V1.Config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.V1.Config.util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;



@Autonomous(name = "Far Auto - 9 Ball - SOMEWHAT COMPATABLE", group = "AA", preselectTeleOp = "v1Teleop")
public class FarAuto  extends OpMode {

    private Follower follower;
    private Timer pathTimer;
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
    private boolean ball_was_present = true;

    private final double shooterDesiredVelo = 1000;
    private Poses.Alliance lastKnownAlliance = null;




    // Tolerance for alignment in radians (approx. 2 degrees)
    // private static final double ALIGN_THRESHOLD = Math.toRadians(2);


    private PathChain travelToShoot, intake3, travelBackToShoot3,intake4, travelBackToShoot4, travelToHumanPlayer;

    public void buildPaths() {
        // --- Alliance-Aware Path Generation ---
        travelToShoot = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.startPoseFar), Poses.get(Poses.shootPositionGoalSide)))
                .setLinearHeadingInterpolation(Poses.get(Poses.startPoseFar).getHeading(), Poses.get(Poses.shootPositionGoalSide).getHeading())

                .build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.shootPositionGoalSide), Poses.get(Poses.Line3ControlPoint),Poses.get(Poses.pickupLine3)))
                .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide).getHeading(), Poses.get(Poses.pickupLine3).getHeading(),0.45)
                .build();

        travelBackToShoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.pickupLine3), Poses.get(Poses.Line3ControlPoint), Poses.get(Poses.shootPositionGoalSide)))
                .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine3).getHeading(), Poses.get(Poses.shootPositionGoalSide).getHeading())
                .build();

        intake4 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.shootPositionGoalSide), Poses.get(Poses.Line4ControlPoint),Poses.get(Poses.pickupLine4)))
                .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide).getHeading(), Poses.get(Poses.pickupLine4).getHeading(),0.45)
                .build();

        travelBackToShoot4 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.pickupLine4), Poses.get(Poses.Line4ControlPoint), Poses.get(Poses.shootPositionGoalSide)))
                .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine4).getHeading(), Poses.get(Poses.shootPositionGoalSide).getHeading())
                .build();
        travelToHumanPlayer = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.shootPositionGoalSide), Poses.get(Poses.endByHumanPlayer)))
                .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide).getHeading(), Poses.get(Poses.shootPositionGoalSide).getHeading())
                .build();


    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Initial Travel to Shoot Position
                intake.intakeRetainBalls();

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
                intake.intakeInDistance();

                if (!follower.isBusy()) {

                    follower.followPath(intake3, true);
                    setPathState();
                }
                break;
            case 3: //go to shoot

                if (!follower.isBusy()) {
                    intake.intakeRetainBalls();
                    follower.followPath(travelBackToShoot3, true);

                    setPathState();
                }
                break;
            case 4: //shoot
                if (!follower.isBusy()) {
                    shootBalls();
                }
                break;
            case 5: //go to intake
                intake.intakeInDistance();

                if (!follower.isBusy()) {

                    follower.followPath(intake4, true);
                    setPathState();
                }
                break;
            case 6: //go to shoot

                if (!follower.isBusy()) {
                    intake.intakeRetainBalls();
                    follower.followPath(travelBackToShoot4, true);

                    setPathState();
                }
                break;
            case 7: //shoot
                if (!follower.isBusy()) {
                    shootBalls();
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(travelToHumanPlayer,true);
                    setPathState();
                }
                break;


            default: // End State (-1)
                shooter.stopShooter();
                intake.fullIntakeIdle();
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
        intake.fullIntakeIdle();
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
        follower.setStartingPose(Poses.get(Poses.startPoseFar));

        if (Poses.getAlliance() != lastKnownAlliance) {

            buildPaths();

            lastKnownAlliance = Poses.getAlliance();
            telemetry.addData("STATUS", "Paths Rebuilt for " + lastKnownAlliance);
        }


        telemetry.addData("Alliance Set", Poses.getAlliance());
        telemetry.addData("Start Pose", Poses.get(Poses.startPoseFar));
        telemetry.update();
    }

    @Override
    public void start() {
        //buildPaths();
        pathTimer.resetTimer();
        setPathState(0);
    }

    public void shootBalls(){
        shooter.shoot(shooterDesiredVelo);
        double currentVelo = shooter.getCurrentVelo();

        boolean isShooterReady = currentVelo >= shooterDesiredVelo - 25 && currentVelo <= shooterDesiredVelo + 55;

        if (isShooterReady) {
            intake.intakeIn();
            intake.setServoPower(1);
        }

        boolean ball_is_present = (intake.getDistanceMM() <= 60);

        if (ball_was_present && !ball_is_present) {
            shotCounter++;
        }

        ball_was_present = ball_is_present;

        if (shotCounter >= 3 || pathTimer.getElapsedTimeSeconds() >= 7) {
            shooter.stopShooter();
            intake.setServoPower(0);
            intake.fullIntakeIdle();
            shotCounter = 0;
            setPathState();
        }


    }



}