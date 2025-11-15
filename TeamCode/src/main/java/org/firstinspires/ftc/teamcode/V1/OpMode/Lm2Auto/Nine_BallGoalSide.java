package org.firstinspires.ftc.teamcode.V1.OpMode.Lm2Auto;

import com.bylazar.configurables.annotations.Configurable;
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
@Configurable
@Autonomous(name = "GS 9 Ball AP Friendly", group = "AA", preselectTeleOp = "v1Teleop")
public class  Nine_BallGoalSide extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private Shooter shooter;
    private Intake intake;
    private LimelightCamera limelight;
    private int pathState;
    private int shotCounter = 0;

    private static  double shooterDesiredVelo = 1043;
    private static final double shooterDesiredDipVelo = 955;
    private Poses.Alliance lastKnownAlliance = null;

    boolean shooterHasSpunUp = false;
    boolean shooterBelow = false;;

    long lastDipTime = 0;
    final long DIP_COOLDOWN_MS = 120;  // minimum time between shots




    // Tolerance for alignment in radians (approx. 2 degrees)
    // private static final double ALIGN_THRESHOLD = Math.toRadians(2);


    private PathChain travelToShoot,  intake1, travelBackToShoot1, intake2, travelBackToShoot2,  intake3, travelBackToShoot3, travelToGate;

    public void buildPaths() {
        // --- Alliance-Aware Path Generation ---
        travelToShoot = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.startPoseGoalSide), Poses.get(Poses.shootPositionGoalSide2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.startPoseGoalSide).getHeading(), Poses.get(Poses.shootPositionGoalSide2).getHeading())

                .build();

        // Path 2: Travel from Shooting Position to Intake Position
        intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.shootPositionGoalSide2), Poses.get(Poses.controlPointLine1ForShootPose2),Poses.get(Poses.pickupLine1)))
                .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide2).getHeading(), Poses.get(Poses.pickupLine1).getHeading(),0.25)
                .build();


        travelBackToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.pickupLine1), Poses.get(Poses.shootPositionGoalSide2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine1).getHeading(), Poses.get(Poses.shootPositionGoalSide2).getHeading())
                .build();
        intake2 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.shootPositionGoalSide2), Poses.get(Poses.line2ControlPoint),Poses.get(Poses.pickupLine2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide2).getHeading(), Poses.get(Poses.pickupLine2).getHeading(),0.45)
                .build();
        travelBackToShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.pickupLine2), Poses.get(Poses.line2ControlPoint), Poses.get(Poses.shootPositionGoalSide2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine2).getHeading(), Poses.get(Poses.shootPositionGoalSide2).getHeading(),0.8)
                .build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.shootPositionGoalSide2), Poses.get(Poses.Line3ControlPoint),Poses.get(Poses.pickupLine3)))
                .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide2).getHeading(), Poses.get(Poses.pickupLine3).getHeading(),0.45)
                .build();

        travelBackToShoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(Poses.get(Poses.pickupLine3), Poses.get(Poses.Line3ControlPoint), Poses.get(Poses.shootPositionGoalSide2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine3).getHeading(), Poses.get(Poses.shootPositionGoalSide2).getHeading())
                .build();
        travelToGate = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.shootPositionGoalSide2), Poses.get(Poses.lineupAtGate)))
                .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide2).getHeading(), Poses.get(Poses.lineupAtGate).getHeading())
                .build();
    }


    public void autonomousPathUpdate() {
        follower.setMaxPower(0.8);
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
                    shootBalls(shooterDesiredVelo,3,10,shooterDesiredDipVelo);
                }
                break;
            case 2: //go to intake
                intake.intakeIn();
//                intake.intakeIn();

                if (!follower.isBusy()) {

                    follower.followPath(intake1, true);
                    setPathState();
                }
                break;
            case 3: //go to shoot
                if (pathTimer.getElapsedTimeSeconds() <= 0.5) {
                    intake.setServoPower(-1);
                } else {
                    intake.setServoPower(0);

                }

                if (!follower.isBusy()) {
                    follower.followPath(travelBackToShoot1, true);

                    setPathState();
                }
                break;
            case 4: //shoot
                if (!follower.isBusy()) {
                    ;
                   shootBalls(shooterDesiredVelo,3,6,shooterDesiredDipVelo);
                }
                break;
            case 5: //go to intake
                intake.intakeIn();
//                intake.intakeIn();

                if (!follower.isBusy()) {

                    follower.followPath(intake2, true);
                    setPathState();
                }
                break;
            case 6: //go to shoot
                if (pathTimer.getElapsedTimeSeconds() <= 0.5) {
                intake.setServoPower(-1);
            } else {
                intake.setServoPower(0);

            }
                if (!follower.isBusy()) {
                    intake.intakeIn();
                    follower.followPath(travelBackToShoot2, true);

                    setPathState();
                }
                break;
            case 7: //shoot
                if (!follower.isBusy()) {

                    shootBalls(shooterDesiredVelo,3,6,shooterDesiredDipVelo);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(travelToGate,true);
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
        telemetry.addLine("");
        telemetry.addData("Shot Counter: ", shotCounter);

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


        if (Poses.getAlliance() != lastKnownAlliance) {
            follower.setStartingPose(Poses.get(Poses.startPoseGoalSide));
            buildPaths();

            lastKnownAlliance = Poses.getAlliance();
            telemetry.addData("STATUS", "Paths Rebuilt for " + lastKnownAlliance);
            telemetry.addLine("");
        }


        telemetry.addLine("--- Alliance Selector ---");
        telemetry.addLine("D-pad UP → RED | D-pad DOWN → BLUE");
        telemetry.addLine("");
        telemetry.addData("Alliance Set", Poses.getAlliance());
        telemetry.addData("Start Pose", Poses.get(Poses.startPoseGoalSide));
        telemetry.addData("Distance Sensor", intake.getDistanceMM());
        telemetry.update();
    }

    @Override
    public void start() {
        //buildPaths();
        pathTimer.resetTimer();
        setPathState(0);
    }

//    public void shootBalls(double velo, int numShots){
//        shooter.shoot(velo);
//        double currentVelo = shooter.getCurrentVelo();
//
//        boolean isShooterReady = currentVelo >= velo - 25 && currentVelo <= velo + 55;
//
//        if (isShooterReady) {
//            intake.intakeIn();
//            intake.setServoPower(1);
//        }
//
//        boolean ball_is_present = (intake.isRightDistance());
//
//        if (ball_was_present && !ball_is_present) {
//            shotCounter++;
//        }
//
//        ball_was_present = ball_is_present;
//
//
//
//        if (shotCounter >= numShots  && pathTimer.getElapsedTimeSeconds() <=3){
//            shotCounter = 0;
//        }
//
//        if (shotCounter >= numShots || pathTimer.getElapsedTimeSeconds() >= 5) {
//            shooter.stopShooter();
//            intake.setServoPower(0);
//            intake.fullIntakeIdle();
//            shotCounter = 0;
//            setPathState();
//        }
//
//
//    }

    public void shootBalls(double targetVelo, int numShots, int timerValue, double veloDipValue) {

        shooter.shoot(targetVelo);
        double currentVelo = shooter.getCurrentVelo();

        long now = System.currentTimeMillis();


        if (currentVelo >= veloDipValue) {
            shooterHasSpunUp = true;
        }


        if (shooterHasSpunUp &&
                currentVelo < veloDipValue &&
                !shooterBelow &&
                now - lastDipTime > DIP_COOLDOWN_MS) {

            shotCounter++;
            shooterBelow = true;
            lastDipTime = now;
        }

        if (currentVelo >= veloDipValue) {
            shooterBelow = false;
        }

        // ------------------------------------
        // 3) Feed balls when shooter ready
        // ------------------------------------
        boolean ready = currentVelo >= targetVelo - 25 && currentVelo <= targetVelo + 55;

        if (ready) {
            intake.intakeIn();
            intake.setServoPower(1);
        }
        else if (!ready){
            intake.intakeRetainBalls();
            intake.setServoPower(0);
        }


        if (shotCounter >= numShots) {

            if (currentVelo >= veloDipValue) {
                shooter.stopShooter();
                intake.setServoPower(0);
                intake.fullIntakeIdle();

                shotCounter = 0;
                shooterHasSpunUp = false;

                setPathState();
                return;
            }
        }


        if (pathTimer.getElapsedTimeSeconds() >= timerValue) {
            shooter.stopShooter();
            intake.setServoPower(0);
            intake.fullIntakeIdle();

            shotCounter = 0;
            shooterHasSpunUp = false;
            setPathState();
        }
    }


}

