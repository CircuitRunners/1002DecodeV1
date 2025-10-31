package org.firstinspires.ftc.teamcode.V1.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;

import java.util.List;

import org.firstinspires.ftc.teamcode.V1.Config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.V1.Config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.V1.Config.util.HeadingAutoAligner;
import org.firstinspires.ftc.teamcode.V1.Config.util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

    @Autonomous(name = "Goal Side Auto Clean", preselectTeleOp = "v1Teleop")
    public class HopefullyLessSketchGoalSideAuto extends OpMode {
            private Follower follower;
            private Timer pathTimer;

            private HeadingAutoAligner aligner;
            private Shooter shooter;
            private Intake intake;

            private int pathState;
            private int ballsToShoot = 0;
            private int shotCounter = 0;

            private Poses.Alliance lastKnownAlliance = null;

            // --- State Constants ---
            private static final int STATE_END = -1;

            // --- Tuning Constants ---
            private final double SHOOTER_TARGET_VELO = 1000;
            private final double RAMP_UP_TIME = 0.75; //idk actual value rn
            private final double TIME_PER_SHOT = 2.0; //idk actual value rn its prob more
            private final double POST_VOLLEY_DELAY = 0.25; //idk actual value rn

            private PathChain travelToShoot, travelToIntake1, intake1, travelBackToShoot1, travelToIntake2, intake2, travelBackToShoot2, travelToGate;

            public void buildPaths() {

                // Path 1: Travel from Start to a Shooting Position
                travelToShoot = follower.pathBuilder()
                        .addPath(new BezierLine(Poses.get(Poses.startPoseGoalSide), Poses.get(Poses.shootPositionGoalSide)))
                        .setLinearHeadingInterpolation(0, Poses.get(Poses.shootPositionGoalSide).getHeading())
                        .build();

                // --- Cycle 1 ---
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

                // --- Cycle 2 ---
                travelToIntake2 = follower.pathBuilder()
                        .addPath(new BezierLine(Poses.get(Poses.shootPositionGoalSide2), Poses.get(Poses.lineupLine2)))
                        .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide2).getHeading(), Poses.get(Poses.lineupLine2).getHeading())
                        .build();

                intake2 = follower.pathBuilder()
                        .addPath(new BezierLine(Poses.get(Poses.lineupLine2), Poses.get(Poses.pickupLine2)))
                        .setConstantHeadingInterpolation(Poses.get(Poses.pickupLine2).getHeading())
                        .build();

                travelBackToShoot2 = follower.pathBuilder()
                        .addPath(new BezierCurve(Poses.get(Poses.pickupLine2), Poses.get(Poses.backToShoot2ControlPoint), Poses.get(Poses.shootPositionGoalSide2)))
                        .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine2).getHeading(), Poses.get(Poses.shootPositionGoalSide2).getHeading())
                        .build();

                // --- Parking ---
                travelToGate = follower.pathBuilder()
                        .addPath(new BezierLine(Poses.get(Poses.shootPositionGoalSide2), Poses.get(Poses.lineupAtGate)))
                        .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide2).getHeading(), Poses.get(Poses.lineupAtGate).getHeading())
                        .build();

                if (Poses.getAlliance() == Poses.Alliance.RED) {
                    aligner = new HeadingAutoAligner(12, 132);
                } else {
                    aligner = new HeadingAutoAligner(132, 132);
                }
            }


            public void autonomousPathUpdate() {
                // This is the shoot EXECUTION handler. It runs when a shoot state (1, 5, 9) is active.
                if (pathState == 1 || pathState == 5 || pathState == 9) {
                    shootVolley();
                    return;
                }

                // The switch handles path completion and sequence INITIATION.
                switch (pathState) {
                    case 0:
                        follower.followPath(travelToShoot, true);
                        startShootVolley(3);
                        setPathState();
                        break;

                    case 1:
                        // State 1 is the shoot execution state, handled by the 'if' block above.
                        // It transitions to 2 when shootVolley() is finished.
                        break;

                    case 2:

                        if (!follower.isBusy()) {
                            shooter.stopShooter();
                            intake.intakeOut();
                            intake.setServoPower(0);
                            follower.followPath(travelToIntake1, true);
                            setPathState();
                        }
                        break;

                    case 3:

                        if (!follower.isBusy()) {
                            intake.intakeIn();
                            intake.setServoPower(0); // Servo off while intaking
                            follower.followPath(intake1, true);
                            setPathState();
                        }
                        break;

                    case 4:

                        if (!follower.isBusy()) {
                            intake.intakeRetainBalls();
                            intake.setServoPower(0);
                            follower.followPath(travelBackToShoot1, true);
                            startShootVolley(3);
                            setPathState();
                        }
                        break;

                    case 5:
                        // State 5 is the shoot execution state, handled by the 'if' block above.
                        // It transitions to 6 when shootVolley() is finished.
                        break;

                    case 6:

                        if (!follower.isBusy()) {
                            shooter.stopShooter();
                            intake.intakeOut();
                            intake.setServoPower(0);
                            follower.followPath(travelToIntake2, true);
                            setPathState();
                        }
                        break;

                    case 7:

                        if (!follower.isBusy()) {
                            intake.intakeIn();
                            intake.setServoPower(0); // Servo off while intaking
                            follower.followPath(intake2, true);
                            setPathState();
                        }
                        break;

                    case 8:

                        if (!follower.isBusy()) {
                            intake.intakeRetainBalls();
                            intake.setServoPower(0);
                            follower.followPath(travelBackToShoot2, true);
                            startShootVolley(3);
                            setPathState(9);
                        }
                        break;

                    case 9:
                        // State 9 is the shoot execution state, handled by the 'if' block above.
                        // It transitions to 10 when shootVolley() is finished.
                        break;

                    case 10:

                        if (!follower.isBusy()) {
                            shooter.stopShooter();
                            intake.intakeIdle();
                            intake.setServoPower(0);
                            follower.followPath(travelToGate, true);
                            setPathState(STATE_END);
                        }
                        break;

                    case STATE_END:
                        shooter.stopShooter();
                        intake.intakeIdle();
                        intake.setServoPower(0); // Ensure servo is off at the end
                        if (!follower.isBusy()) {
                            requestOpModeStop();
                        }
                        break;
                }
            }

            public void startShootVolley(int count) {
                ballsToShoot = count;
                shooter.shoot(SHOOTER_TARGET_VELO);
                pathTimer.resetTimer();
                shotCounter = 0;
                telemetry.addData("Shooting Volley", "Starting - To Shoot Count: " + count);
            }

        public void shootVolley() {
            double elapsedTime = pathTimer.getElapsedTimeSeconds();
            boolean shooterReady = shooter.getCurrentVelo() >= SHOOTER_TARGET_VELO - 50;

            // 1. VOLLEY COMPLETE PHASE
            if (ballsToShoot == 0) {
                intake.setServoPower(0); // Ensure servo is off

                if (pathTimer.getElapsedTimeSeconds() >= POST_VOLLEY_DELAY) {
                    shooter.stopShooter();
                    intake.intakeIdle();
                    setPathState();
                    telemetry.addData("Shooting Volley", "Complete, Moving to State " + pathState);
                } else {
                    shooter.stopShooter();
                    intake.intakeIdle();
                }
            }

            // 2. RAMP UP & HOLD PHASE
            else if (elapsedTime < RAMP_UP_TIME || !shooterReady) {
                intake.intakeIdle();
                intake.setServoPower(0);
            }

            // 3. FEEDING PHASE
            else {
                double currentShotTime = RAMP_UP_TIME + (TIME_PER_SHOT * shotCounter);

                // Time to start/continue feeding the current shot
                if (elapsedTime >= currentShotTime) {
                    intake.intakeIn();
                    intake.setServoPower(1); // Servo ON to feed the ball
                    telemetry.addData("FEEDING", "Shot " + (shotCounter + 1) + " / " + (int)(shotCounter + ballsToShoot));

                    // Time to complete the current shot and move to the next one
                    if (elapsedTime >= currentShotTime + TIME_PER_SHOT) {
                        shotCounter++;
                        ballsToShoot--;
                        if (ballsToShoot == 0) {
                            pathTimer.resetTimer();
                        }
                    }
                } else {
                    // Between shots: servo in the non-feeding state.
                    intake.setServoPower(0);
                }
            }

            telemetry.addData("Shooter Status", shooterReady ? "READY" : "RAMPING");
            telemetry.addData("Balls Remaining", ballsToShoot);
            telemetry.addData("Shot Index", shotCounter);
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
                telemetry.addData("Shooter Velocity", shooter.getCurrentVelo());

                autonomousPathUpdate();

                telemetry.update();
            }

            @Override
            public void stop() {
                shooter.stopShooter();
                intake.intakeIdle();
                intake.setServoPower(0);
                Poses.savePose(follower.getPose());
            }

            @Override
            public void init() {
                List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
                for (LynxModule hub : allHubs) {
                    hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
                }

                pathTimer = new Timer();
                follower = Constants.createFollower(hardwareMap);

                intake = new Intake(hardwareMap, telemetry);
                shooter = new Shooter(hardwareMap, telemetry);

                follower.setStartingPose(Poses.get(Poses.startPoseGoalSide));
                buildPaths();
                lastKnownAlliance = Poses.getAlliance();

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
                pathTimer.resetTimer();
                setPathState(0);
            }
    }


