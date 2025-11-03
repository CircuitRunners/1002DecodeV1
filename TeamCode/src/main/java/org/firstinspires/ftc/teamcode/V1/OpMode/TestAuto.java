package org.firstinspires.ftc.teamcode.V1.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.V1.Config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.V1.Config.subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.V1.Config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.V1.Config.util.HeadingAutoAligner;
import org.firstinspires.ftc.teamcode.V1.Config.util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;




    @Autonomous(name = "test", preselectTeleOp = "v1Teleop")
    public class TestAuto extends OpMode {

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







        private PathChain travelToMiddle;

        public void buildPaths() {
            travelToMiddle = follower.pathBuilder()
                    .addPath(new BezierLine(Poses.get(Poses.startPoseGoalSide), new Pose(72,72,Math.toRadians(90))))
                    .setLinearHeadingInterpolation(0,Math.toRadians(90))

                    .build();
        }


        public void autonomousPathUpdate() {
            switch (pathState) {
                case 0: // Initial Travel to Shoot Position
//                intake.intakeRetainBalls();
//                intake.intakeIn();
                    follower.setMaxPower(1);
                    if(follower.getCurrentTValue() >=0.4){
                        //limelight.getTagId();

                    }
                    if (!follower.isBusy()) {
                        follower.followPath(travelToMiddle, true);
                        setPathState();
                    }
                    break;


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
            telemetry.addData("Path State", pathState);

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

        //LESS CHOPPED SHOOT BALLS HERE:
        public void shootBalls(int seconds){
            // Start the shooter flywheel immediately.
            shooter.shoot(shooterDesiredVelo);
            double currentVelo = shooter.getCurrentVelo();

            // Assume shooting has NOT started until we meet the velocity target.
            boolean isShooterReady = currentVelo >= shooterDesiredVelo - 25 && currentVelo <= shooterDesiredVelo + 55;

            if (isShooterReady) {
                intake.intakeIn();
                intake.setServoPower(1);
            }
//
            // --- Move to next path after timeout ---
            if (pathTimer.getElapsedTimeSeconds() >= seconds) {
                shooter.stopShooter();
                intake.setServoPower(0);
//            intake.intakeIdle();
                shotCounter = 0;
                setPathState();
            }

            lastShooterVelo = currentVelo;
        }



    }

