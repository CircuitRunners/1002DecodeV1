package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;


@Disabled
@TeleOp(name = "IntakeTest")
public class IntakeTest extends OpMode {

    private GamepadEx player1;
    private DcMotorEx intake;
    private CRServo servoIntake;

    private static final int TICKS_PER_REV = 537; // Example: goBILDA 312 RPM Yellow Jacket
    private double target = 800;   // starting RPM
    private double servoPower = 0.0;
    boolean reverseServo = false;
    private double intakePower = 0.0;
    private double increment = 50;  // increment/decrement per button press

    final double STOP_SPEED = 0.0;

    @Override
    public void init(){
        player1 = new GamepadEx(gamepad1);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        servoIntake = hardwareMap.get(CRServo.class, "feederWheel");

        telemetry.addLine("Ready!");
        telemetry.update();
    }

    @Override
    public void loop() {
        player1.readButtons();
        double ticksPerSecond = (target / 60.0) * TICKS_PER_REV;

        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            target += increment;

        } else if (player1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            target -= increment;

        }

        if (player1.wasJustPressed((GamepadKeys.Button.SQUARE))) {
            intakePower = STOP_SPEED;

        } else if (player1.wasJustPressed((GamepadKeys.Button.CIRCLE))) {
            intakePower = ticksPerSecond;

        }

        if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) {
            if (reverseServo) {
                servoPower = -servoPower;

            } else {
                servoPower = 1.0;
                reverseServo = true;
            }

        } else if (player1.wasJustPressed(GamepadKeys.Button.X)) {
            servoPower = 0.5;
            reverseServo = false;
        }

        servoIntake.setPower(servoPower);
        intake.setVelocity(intakePower);

        // Convert RPM to ticks per second

        telemetry.addData("Target RPM", target);
        telemetry.addData("Intake RPM", intake.getVelocity() * 60.0 / TICKS_PER_REV);

        telemetry.addData("Target TPS", ticksPerSecond);
        telemetry.addData("Intake TPS", intake.getVelocity());

        telemetry.addData("Servo Power", servoPower); // servoIntake.getPower()

        telemetry.update();
    }

}
