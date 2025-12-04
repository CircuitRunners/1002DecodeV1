package org.firstinspires.ftc.teamcode.V1.OpMode.Lm2Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.V1.Config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.V1.Config.subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.V1.Config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.V1.Config.util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
@Configurable
@Autonomous(name = "TwelveBall")
public class  TwelveBallAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private Shooter shooter;
    private Intake intake;
    private LimelightCamera limelight;
    private int pathState;
    private int shotCounter = 0;

    private static final double shooterDesiredVelo = 1020.5;
    private static final double shooterDesiredDipVelo = 935;
    private double desiredVelo;
    private thisAlliance lastKnownAlliance = null;

    boolean shooterHasSpunUp = false;
    boolean shooterBelow = false;;

    long lastDipTime = 0;
    final long DIP_COOLDOWN_MS = 120;  // minimum time between shots




    private enum thisAlliance {RED, BLUE}
    private static class thisAlliancePose {
        private final Pose bluePose;
        private final Pose redPose;

        private thisAlliancePose(Pose bluePose, Pose redPose) {
            this.redPose = redPose;
            this.bluePose = bluePose;
        }

        private Pose getPose(thisAlliance alliance) {
            return (alliance == thisAlliance.RED) ? redPose : bluePose;
        }


    }

    private thisAlliance currentAlliance = thisAlliance.RED;

//    private static thisAlliance getAlliance() {
//        return currentAlliance;
//    }
//
//    private static void setAlliance(thisAlliance Alliance) {
//        currentAlliance = Alliance;
//    }

    private void updateAlliance(Gamepad g) {
        if (g.dpad_up) {
            currentAlliance = thisAlliance.RED;
        }
        else if (g.dpad_down) {
            currentAlliance = thisAlliance.BLUE;
        }
    }

    private thisAlliance getAlliance() {
        return currentAlliance;
    }

    private PathChain travelToShoot,  intake1, travelBackToShoot1, intake2, travelBackToShoot2,  intake3, travelBackToShoot3, travelToGate;
    private static final thisAlliancePose startPoseGoalSide = new thisAlliancePose(
            new Pose(32, 135.5, Math.toRadians(180)),
            new Pose(144-32, 135.5, Math.toRadians(0))
    );
    private static final thisAlliancePose shootPositionGoalSide = new thisAlliancePose(
            new Pose(40, 104, Math.toRadians(140)),
            new Pose(144-40, 104, Math.toRadians(40))
    );
    private static final thisAlliancePose controlPickupLine1 = new thisAlliancePose(
            new Pose(90, 81.5),
            new Pose(144-90, 81.5)
    );
    private static final thisAlliancePose pickupLine1 = new thisAlliancePose(
            new Pose(13, 81.5, Math.toRadians(180)),
            new Pose(144-13, 81.5, Math.toRadians(0))
    );
    private static final thisAlliancePose controlPickupLine2 = new thisAlliancePose(
            new Pose(90, 51),
            new Pose(144-90, 51)
    );
    private static final thisAlliancePose pickupLine2 = new thisAlliancePose(
            new Pose(11, 55, Math.toRadians(180)),
            new Pose(144-11, 55, Math.toRadians(0))
    );
    private static final thisAlliancePose controlPickupLine3 = new thisAlliancePose(
            new Pose(90, 34),
            new Pose(144-90, 34)
    );
    private static final thisAlliancePose pickupLine3 = new thisAlliancePose(
            new Pose(11, 36, Math.toRadians(180)),
            new Pose(144-11, 36, Math.toRadians(0))
    );
    private static final thisAlliancePose controlPickupLineToShoot = new thisAlliancePose(
            new Pose(70, 81),
            new Pose(144-70, 81)
    );
    private static final thisAlliancePose lineupAtGate = new thisAlliancePose(
            new Pose(20, 65, Math.toRadians(270)),
            new Pose(144-20, 65, Math.toRadians(270))
    );
    public void buildPaths() {
        // --- Alliance-Aware Path Generation ---
        travelToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPoseGoalSide.getPose(currentAlliance), shootPositionGoalSide.getPose(currentAlliance)))
                .setLinearHeadingInterpolation(startPoseGoalSide.getPose(currentAlliance).getHeading(), shootPositionGoalSide.getPose(currentAlliance).getHeading())

                .build();

        // Path 2: Travel from Shooting Position to Intake Position
        intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPositionGoalSide.getPose(currentAlliance), controlPickupLine1.getPose(currentAlliance), pickupLine1.getPose(currentAlliance)))
                .setLinearHeadingInterpolation(shootPositionGoalSide.getPose(currentAlliance).getHeading(), pickupLine1.getPose(currentAlliance).getHeading())
                .build();

        travelBackToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(pickupLine1.getPose(currentAlliance), shootPositionGoalSide.getPose(currentAlliance)))
                .setLinearHeadingInterpolation(pickupLine1.getPose(currentAlliance).getHeading(), shootPositionGoalSide.getPose(currentAlliance).getHeading())
                .build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPositionGoalSide.getPose(currentAlliance), controlPickupLine2.getPose(currentAlliance), pickupLine2.getPose(currentAlliance)))
                .setLinearHeadingInterpolation(shootPositionGoalSide.getPose(currentAlliance).getHeading(), pickupLine2.getPose(currentAlliance).getHeading())
                .build();

        travelBackToShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickupLine2.getPose(currentAlliance), controlPickupLineToShoot.getPose(currentAlliance), shootPositionGoalSide.getPose(currentAlliance)))
                .setLinearHeadingInterpolation(pickupLine2.getPose(currentAlliance).getHeading(), shootPositionGoalSide.getPose(currentAlliance).getHeading())
                .build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPositionGoalSide.getPose(currentAlliance), controlPickupLine3.getPose(currentAlliance), pickupLine3.getPose(currentAlliance)))
                .setLinearHeadingInterpolation(shootPositionGoalSide.getPose(currentAlliance).getHeading(), pickupLine3.getPose(currentAlliance).getHeading())
                .build();

        travelBackToShoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickupLine3.getPose(currentAlliance), controlPickupLineToShoot.getPose(currentAlliance), shootPositionGoalSide.getPose(currentAlliance)))
                .setLinearHeadingInterpolation(pickupLine3.getPose(currentAlliance).getHeading(), shootPositionGoalSide.getPose(currentAlliance).getHeading())
                .build();

        travelToGate = follower.pathBuilder()
                .addPath(new BezierLine(shootPositionGoalSide.getPose(currentAlliance), lineupAtGate.getPose(currentAlliance)))
                .setLinearHeadingInterpolation(shootPositionGoalSide.getPose(currentAlliance).getHeading(), lineupAtGate.getPose(currentAlliance).getHeading())
                .build();
    }


    public void autonomousPathUpdate() {
        //follower.setMaxPower(0.8);
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
                    if (currentAlliance == thisAlliance.BLUE) {
                        desiredVelo = shooter.calculateFlywheelVelocity(limelight.calculateDistanceToGoal(follower.getPose().getX(), follower.getPose().getY(), 12, 137));
                    }
                    else if (currentAlliance == thisAlliance.RED) {
                        desiredVelo = shooter.calculateFlywheelVelocity(limelight.calculateDistanceToGoal(follower.getPose().getX(), follower.getPose().getY(), 132, 137));
                    }
                    //shootBalls(shooterDesiredVelo,3,10,shooterDesiredDipVelo);
                    shootBalls(desiredVelo,3,6,shooterDesiredDipVelo);

                }
                break;

            case 2: //go to intake
                intake.intakeIn();

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
                    if (currentAlliance == thisAlliance.BLUE) {
                        desiredVelo = shooter.calculateFlywheelVelocity(limelight.calculateDistanceToGoal(follower.getPose().getX(), follower.getPose().getY(), 12, 137));
                    }
                    else if (currentAlliance == thisAlliance.RED) {
                        desiredVelo = shooter.calculateFlywheelVelocity(limelight.calculateDistanceToGoal(follower.getPose().getX(), follower.getPose().getY(), 132, 137));
                    }                    //shootBalls(shooterDesiredVelo,3,6,shooterDesiredDipVelo);
                    shootBalls(desiredVelo,3,6,shooterDesiredDipVelo);
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
                    if (currentAlliance == thisAlliance.BLUE) {
                        desiredVelo = shooter.calculateFlywheelVelocity(limelight.calculateDistanceToGoal(follower.getPose().getX(), follower.getPose().getY(), 12, 137));
                    }
                    else if (currentAlliance == thisAlliance.RED) {
                        desiredVelo = shooter.calculateFlywheelVelocity(limelight.calculateDistanceToGoal(follower.getPose().getX(), follower.getPose().getY(), 132, 137));
                    }                    //shootBalls(shooterDesiredVelo,3,6,shooterDesiredDipVelo);
                    shootBalls(desiredVelo,3,6,shooterDesiredDipVelo);
                }
                break;

            case 8: //go to intake
                intake.intakeIn();
//                intake.intakeIn();

                if (!follower.isBusy()) {

                    follower.followPath(intake3, true);
                    setPathState();
                }
                break;

            case 9: //go to shoot
                if (pathTimer.getElapsedTimeSeconds() <= 0.2) {
                    intake.setServoPower(-1);
                } else {
                    intake.setServoPower(0);

                }
                if (!follower.isBusy()) {
                    intake.intakeIn();
                    follower.followPath(travelBackToShoot3, true);

                    setPathState(10);
                }
                break;

            case 10: //shoot

                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    pathTimer.resetTimer();
                }

                if (!follower.isBusy()) {
                    if (currentAlliance == thisAlliance.BLUE) {
                        desiredVelo = shooter.calculateFlywheelVelocity(limelight.calculateDistanceToGoal(follower.getPose().getX(), follower.getPose().getY(), 12, 137));
                    }
                    else if (currentAlliance == thisAlliance.RED) {
                        desiredVelo = shooter.calculateFlywheelVelocity(limelight.calculateDistanceToGoal(follower.getPose().getX(), follower.getPose().getY(), 132, 137));
                    }                    //shootBalls(shooterDesiredVelo,3,6,shooterDesiredDipVelo);
                    shootBalls(desiredVelo,3,10,shooterDesiredDipVelo);
                }
                break;

            case 11: //go to gate
                if (!follower.isBusy()) {
                    follower.followPath(travelToGate,true);
                    setPathState();
                }
                break;

            default: // End State (-1)
                if (!follower.isBusy()) {
                    shooter.stopShooter();
                    intake.fullIntakeIdle();

                    telemetry.addLine("ERROR: INVALID PATH STATE");
                    telemetry.update();
                    //requestOpModeStop();
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
        telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());


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
        limelight = new LimelightCamera(hardwareMap);
        limelight.limelightCamera.start();
//
//        BallOrder detected = limelight.detectBallOrder();
//
//        telemetry.addData("Ball Order", detected);
        telemetry.update();
    }

    @Override
    public void init_loop() {


        updateAlliance(gamepad1);


        if (getAlliance() != lastKnownAlliance) {
            follower.setStartingPose(startPoseGoalSide.getPose(currentAlliance));
            buildPaths();

            lastKnownAlliance = getAlliance();
            telemetry.addData("STATUS", "Paths Rebuilt for " + lastKnownAlliance);
            telemetry.addLine("");
        }


        telemetry.addLine("--- Alliance Selector ---");
        telemetry.addLine("D-pad UP → RED | D-pad DOWN → BLUE");
        telemetry.addLine("");
        telemetry.addData("Alliance Set", getAlliance());
        telemetry.addData("Start Pose", startPoseGoalSide.getPose(currentAlliance));
        telemetry.addData("Distance Sensor", intake.getDistanceMM());
        telemetry.update();
    }

    @Override
    public void start() {
//        buildPaths();
//        follower.setStartingPose(startPoseGoalSide.getPose(currentAlliance));
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
            telemetry.addLine("Timer ran out");
            telemetry.update();

            shotCounter = 0;
            shooterHasSpunUp = false;
            setPathState();
        }
    }


}

