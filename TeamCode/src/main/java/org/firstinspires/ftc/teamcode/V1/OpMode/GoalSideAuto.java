package org.firstinspires.ftc.teamcode.V1.OpMode;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.lynx.LynxModule;


import java.util.List;

import org.firstinspires.ftc.teamcode.V1.Config.util.HeadingAutoAligner;
import org.firstinspires.ftc.teamcode.V1.Config.util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;



@Autonomous(name = "Goal Side Auto", preselectTeleOp = "v1Teleop")
public class GoalSideAuto extends OpMode {

        private Follower follower;
        private Timer pathTimer;
       // private ShooterIntake shooterIntake;
        private HeadingAutoAligner aligner;
        private int pathState;
        private int shotCounter = 0;

        private double power;

        // Tolerance for alignment in radians (approx. 2 degrees)
        private static final double ALIGN_THRESHOLD = Math.toRadians(2);

        // Generic paths for a shooting sequence
        private PathChain travelToShoot, travelToIntake1, intake1, travelBackToShoot;

        public void buildPaths() {
            // --- Alliance-Aware Path Generation ---
            // Poses.get(POSE_NAME) ensures the correct RED or BLUE coordinates are used.

            // Path 1: Travel from Start to a Shooting Position
            travelToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(Poses.get(Poses.startPose), Poses.get(Poses.shootPosition)))
                    .setLinearHeadingInterpolation(Poses.get(Poses.startPose).getHeading(), Poses.get(Poses.shootPosition).getHeading())
                    .build();

            // Path 2: Travel from Shooting Position to Intake Position
            travelToIntake1 = follower.pathBuilder()
                    .addPath(new BezierLine(Poses.get(Poses.shootPosition), Poses.get(Poses.lineupLine1)))
                    .setLinearHeadingInterpolation(Poses.get(Poses.startPose).getHeading(), Poses.get(Poses.shootPosition).getHeading())
                    .build();
            intake1 = follower.pathBuilder()
                    .addPath(new BezierLine(Poses.get(Poses.lineupLine1), Poses.get(Poses.pickupLine1)))
                    .setConstantHeadingInterpolation(Poses.get(Poses.pickupLine1).getHeading())
                    .build();

            travelBackToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(Poses.get(Poses.pickupLine1), Poses.get(Poses.shootPosition)))
                    .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine1).getHeading(), Poses.get(Poses.shootPosition).getHeading())
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
                    if (!follower.isBusy()) {
                        follower.followPath(travelToShoot, true);
                        setPathState();
                    }
                    break;
                case 1: // Shooter Spin-up and Align
                    //shooterIntake.startShooterSpinUp();
                   // alignToGoal(); // Handles rotation and transition to next state
                    setPathState();
                    break;
                case 2: // Launch
                   while (shotCounter <3) {
                       //shooterIntake.launch();
                       shotCounter++;
                   }
                    //shooterIntake.stopShooterSpinUp(); // Stop after launch
                    setPathState();
                    break;
                case 3: // Travel to Intake
                    if (!follower.isBusy()) {
                        //shooterIntake.startIntakeRun(); // Start intake while moving

                            follower.followPath(travelToIntake1, true);
                            setPathState();

                    }
                    break;
                case 4: // Intake Run
                    // Intake runs while follower is busy moving\

                    if (!follower.isBusy()) {
                        follower.followPath(intake1, true);

                        setPathState();
                    }
                    break;
                case 5: // Travel Back to Shoot Position
                    if (!follower.isBusy()) {
                        //shooterIntake.stopIntakeRun(); // Stop intake after reaching position
                        follower.followPath(travelBackToShoot, true);
                        setPathState();
                    }
                    break;
                case 6: // Back to Spin-up (Looping)
                    //shooterIntake.startShooterSpinUp();
                   // alignToGoal(); // Handles rotation and transition to next state
                    setPathState();
                    break;
                case 7:
                    while (shotCounter <3) {
                        //shooterIntake.launch();
                        shotCounter++;
                    }
                    //shooterIntake.stopShooterSpinUp(); // Stop after launch
                    setPathState(-1);
                    break;
                default: // End State (-1)
                    //shooterIntake.stopAll();
                    if (!follower.isBusy()) {
                        requestOpModeStop();
                    }
            }
        }

        /**
         * Uses the HeadingAutoAligner to rotate the robot towards the goal.
         * Transitions to the next state (launch) when aligned and shooter is ready.
         */
        public void alignToGoal() {
            double currentHeading = follower.getPose().getHeading();

            // Calculate the rotation power needed to align
            double rotationPower = aligner.getRotationPower(follower.getPose().getX(), follower.getPose().getY(), currentHeading);

            // Use the absolute value of the rotation power as a proxy for error magnitude
            double errorMagnitude = Math.abs(rotationPower);

            // Check if both conditions for launching are met
            if (errorMagnitude < ALIGN_THRESHOLD /*&& shooterIntake.isShooterUpToSpeed()*/) {
               // follower.setRotationPower(0);
                setPathState(); // Move to launch state
            } else {
                // Apply rotation power until aligned
                //follower.setRotationPower(rotationPower);
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
           // telemetry.addData("Shooter Ready", shooterIntake.isShooterUpToSpeed());

            telemetry.update();


            //shooterIntake.update(); // Generic subsystem update
            follower.update();
            autonomousPathUpdate();

        }

        @Override
        public void stop() {
            Poses.savePose(follower.getPose());
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

            // NOTE: ShooterIntake initialization is here
            //shooterIntake = new ShooterIntake(hardwareMap);
        }

        @Override
        public void init_loop() {
            // ESSENTIAL: Call this to read gamepad input (dpad_up/down) and set the alliance
            Poses.updateAlliance(gamepad1, telemetry);



                // Set the alliance-specific starting pose before building paths
                follower.setStartingPose(Poses.get(Poses.startPose));
                buildPaths();


            telemetry.addData("Alliance Set", Poses.getAlliance());
            telemetry.addData("Start Pose", Poses.get(Poses.startPose));
            telemetry.update();
        }

        @Override
        public void start() {
            pathTimer.resetTimer();
            setPathState(0);
        }


    }

// -----------------------------------------------------------------------
