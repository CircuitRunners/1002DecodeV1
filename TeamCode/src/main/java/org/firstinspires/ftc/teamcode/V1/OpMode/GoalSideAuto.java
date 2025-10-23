package org.firstinspires.ftc.teamcode.V1.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.lynx.LynxModule;


import java.util.List;

import org.firstinspires.ftc.teamcode.V1.Config.subsystems.Intake;
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
    private int pathState;
    private int shotCounter = 0;

    private double MINIMUM_SHOOTER_VELO = 1050;
    private double MAXIMUM_SHOOTER_VELO = 1150;


    // Tolerance for alignment in radians (approx. 2 degrees)
   // private static final double ALIGN_THRESHOLD = Math.toRadians(2);


    private PathChain travelToShoot, travelToIntake1, intake1, travelBackToShoot1, travelToIntake2, intake2, travelBackToShoot2, travelToIntake3, intake3, travelBackToShoot3, travelToGate;

    public void buildPaths() {
        // --- Alliance-Aware Path Generation ---
        // Poses.get(POSE_NAME) ensures the correct RED or BLUE coordinates are used.

        // Path 1: Travel from Start to a Shooting Position
        travelToShoot = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.startPoseGoalSide), Poses.get(Poses.shootPositionGoalSide)))
                .setLinearHeadingInterpolation(Poses.get(Poses.startPoseGoalSide).getHeading(), Poses.get(Poses.shootPositionGoalSide).getHeading())
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
                .addPath(new BezierLine(Poses.get(Poses.pickupLine1), Poses.get(Poses.shootPositionGoalSide)))
                .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine1).getHeading(), Poses.get(Poses.shootPositionGoalSide).getHeading())
                .build();
        travelToIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.shootPositionGoalSide), Poses.get(Poses.lineupLine2)))
                .setLinearHeadingInterpolation(Poses.get(Poses.shootPositionGoalSide).getHeading(), Poses.get(Poses.lineupLine2).getHeading())
                .build();
        intake2 = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.lineupLine2), Poses.get(Poses.pickupLine2)))
                .setConstantHeadingInterpolation(Poses.get(Poses.pickupLine2).getHeading())
                .build();

        travelBackToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(Poses.get(Poses.pickupLine2), Poses.get(Poses.shootPositionGoalSide)))
                .setLinearHeadingInterpolation(Poses.get(Poses.pickupLine2).getHeading(), Poses.get(Poses.shootPositionGoalSide).getHeading())
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
                if (!follower.isBusy()) {
                    follower.followPath(travelToShoot, true);
                    setPathState();
                }
                break;
            case 1: // Shooter Shoot
                shooter.shoot(1110);
                //aligner.getRotationPower(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());
                if ((shooter.getCurrentVelo() >= MINIMUM_SHOOTER_VELO) && (shooter.getCurrentVelo() <= MAXIMUM_SHOOTER_VELO)) {
                    intake.intakeIn();
                    intake.setServoPower(1);
                    shotCounter++;
                }
                else{
                    intake.setServoPower(0);
                }

                if(shotCounter >= 3){
                    shotCounter = 0;
                    setPathState();
                }
                break;
            case 2: //go to intake
                shooter.stopShooter();
                if (!follower.isBusy()) {
                    follower.followPath(travelToIntake1, true);
                    setPathState();
                }
                break;
            case 3: //intake
                intake.intakeIn();
                if (!follower.isBusy()) {
                    follower.followPath(intake1, true);

                    setPathState();
                }
                break;
            case 4: //go to shoot
                intake.intakeRetainBalls();
                if (!follower.isBusy()) {
                    follower.followPath(travelBackToShoot1, true);

                    setPathState();
                }
                break;
            case 5: //shoot
                shooter.shoot(1110);
                //aligner.getRotationPower(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());
                if ((shooter.getCurrentVelo() >= MINIMUM_SHOOTER_VELO) && (shooter.getCurrentVelo() <= MAXIMUM_SHOOTER_VELO)) {
                    intake.intakeIn();
                    intake.setServoPower(1);
                    shotCounter++;
                }
                else{
                    intake.setServoPower(0);
                }

                if(shotCounter >= 3){
                    shotCounter = 0;
                    setPathState(-1);
                }
                break;


                //-----------
            //not using this stuff yet
                //-----------


            case 6: //go to intake
                shooter.stopShooter();
                if (!follower.isBusy()) {
                    follower.followPath(travelToIntake2, true);
                    setPathState();
                }
                break;
            case 7: //intake
                intake.intakeIn();
                if (!follower.isBusy()) {
                    follower.followPath(intake2, true);

                    setPathState();
                }
                break;
            case 8: //go to shoot
                intake.intakeRetainBalls();
                if (!follower.isBusy()) {
                    follower.followPath(travelBackToShoot2, true);

                    setPathState();
                }
                break;
            case 9: //shoot
                shooter.shoot(1110);
                //aligner.getRotationPower(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());
                if ((shooter.getCurrentVelo() >= MINIMUM_SHOOTER_VELO) && (shooter.getCurrentVelo() <= MAXIMUM_SHOOTER_VELO)) {
                    intake.intakeIn();
                    intake.setServoPower(1);
                    shotCounter++;
                }
                else{
                    intake.setServoPower(0);
                }

                if(shotCounter >= 3){
                    shotCounter = 0;
                    setPathState();
                }
                break;
            case 10: //go to intake
                shooter.stopShooter();
                if (!follower.isBusy()) {
                    follower.followPath(travelToIntake3, true);
                    setPathState();
                }
                break;
            case 11: //intake
                intake.intakeIn();
                if (!follower.isBusy()) {
                    follower.followPath(intake3, true);

                    setPathState();
                }
                break;
            case 12: //go to shoot
                intake.intakeRetainBalls();
                if (!follower.isBusy()) {
                    follower.followPath(travelBackToShoot3, true);

                    setPathState();
                }
                break;
            case 13: //shoot
                shooter.shoot(1110);
                //aligner.getRotationPower(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());
                if ((shooter.getCurrentVelo() >= MINIMUM_SHOOTER_VELO) && (shooter.getCurrentVelo() <= MAXIMUM_SHOOTER_VELO)) {
                    intake.intakeIn();
                    intake.setServoPower(1);
                    shotCounter++;
                }
                else{
                    intake.setServoPower(0);
                }

                if(shotCounter >= 3){
                    shotCounter = 0;
                    setPathState();
                }
                break;
            case 14:
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
        // telemetry.addData("Shooter Ready", shooterIntake.isShooterUpToSpeed());

        telemetry.update();


        //shooterIntake.update(); // Generic subsystem update
        follower.update();
        autonomousPathUpdate();

    }

    @Override
    public void stop() {
        shooter.stopShooter();
        intake.intakeIdle();
        intake.setServoPower(0);
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

        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
    }

    @Override
    public void init_loop() {
        // ESSENTIAL: Call this to read gamepad input (dpad_up/down) and set the alliance
        Poses.updateAlliance(gamepad1, telemetry);



        // Set the alliance-specific starting pose before building paths
        follower.setStartingPose(Poses.get(Poses.startPoseGoalSide));



        telemetry.addData("Alliance Set", Poses.getAlliance());
        telemetry.addData("Start Pose", Poses.get(Poses.startPoseGoalSide));
        telemetry.update();
    }

    @Override
    public void start() {
        buildPaths();
        pathTimer.resetTimer();
        setPathState(0);
    }

}