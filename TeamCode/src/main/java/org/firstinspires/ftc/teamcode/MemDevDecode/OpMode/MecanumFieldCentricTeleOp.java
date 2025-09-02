package org.firstinspires.ftc.teamcode.MemDevDecode.OpMode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.MemDevDecode.Config.MecanumDrive;

import java.util.Locale;

@Disabled
@TeleOp(name = "Mecanum - Field Centric")
public class MecanumFieldCentricTeleOp extends OpMode {

    private MecanumDrive drive;
    private GamepadEx player1;
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void init(){
        telemetry.addLine("Initializing...");
        telemetry.update();

        player1 = new GamepadEx(gamepad1);

        drive = new MecanumDrive();
        drive.init(hardwareMap);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configurePinpoint();


        telemetry.addLine("Ready!");
        telemetry.update();
    }

    @Override
    public void loop() {
        player1.readButtons();

        double forward = player1.getLeftY();
        double strafe  =  player1.getLeftX();
        double rotate  =  player1.getRightX();
        Pose2D currentPose = driveFieldRelative(forward, strafe, rotate);

        // IMU Reset / Calibrate
        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) {
            pinpoint.resetPosAndIMU();
        }
        if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) {
            pinpoint.recalibrateIMU();
        }

        telemetry.addData("fl power", drive.frontLeftMotor.getPower());
        telemetry.addData("fr Power", drive.frontRightMotor.getPower());
        telemetry.addData("rl Power", drive.backLeftMotor.getPower());
        telemetry.addData("rr Power", drive.backRightMotor.getPower());


        String data = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                currentPose.getX(DistanceUnit.INCH),
                currentPose.getY(DistanceUnit.INCH),
                currentPose.getHeading(AngleUnit.DEGREES)
        );
        telemetry.addData("Position", data);

        telemetry.update();
    }


    /**
     * Configures the GoBilda Pinpoint device.
     */
    private void configurePinpoint() {
        pinpoint.setOffsets(6.03262717 * 25.4, 2.71126772 * 25.4, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.resetPosAndIMU();
    }

    /**
     * Applies field-relative drive using the Pinpoint’s IMU heading.
     */
    private Pose2D driveFieldRelative(double forward, double right, double rotate) {
        pinpoint.update();
        Pose2D pos = pinpoint.getPosition();  // Current position

        double robotAngle = Math.toRadians(pos.getHeading(AngleUnit.DEGREES));

        // Convert driver input into field-centric motion
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        theta = AngleUnit.normalizeRadians(theta - robotAngle);

        // Convert to robot-centric forward & strafe
        double newForward = r * Math.sin(theta);
        double newRight   = r * Math.cos(theta);

        // Drive with final inputs
        drive.drive(newForward, newRight, rotate);
        return pos;
    }

}
