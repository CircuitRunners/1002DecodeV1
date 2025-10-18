package org.firstinspires.ftc.teamcode.Testers;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
@Configurable
@TeleOp
public class ShooterTest extends OpMode {
    public DcMotorEx shooter1;
    public DcMotorEx shooter2;
    public GamepadEx player1;
    public static double LAUNCHER_TARGET_VELOCITY = 1633; //velocity in ticks per second; 28 ticks per revolution
    final double STOP_SPEED = 0.0;

    @Override
    public void init() {
        shooter1 = hardwareMap.get(DcMotorEx.class, "motor1");
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooter2 = hardwareMap.get(DcMotorEx.class, "motor2");
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        player1 = new GamepadEx(gamepad1);
    }

    public void loop() {

        player1.readButtons();

        if (gamepad1.circle) {
            shooter1.setVelocity(LAUNCHER_TARGET_VELOCITY);
            shooter2.setVelocity(LAUNCHER_TARGET_VELOCITY);
        }
        if (gamepad1.square) {
            shooter1.setVelocity((STOP_SPEED));
            shooter2.setVelocity((STOP_SPEED));
        }
        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                LAUNCHER_TARGET_VELOCITY+=100;
            shooter1.setVelocity(LAUNCHER_TARGET_VELOCITY);
            shooter2.setVelocity(LAUNCHER_TARGET_VELOCITY);
        }
        if (player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            LAUNCHER_TARGET_VELOCITY-=100;
            shooter1.setVelocity(LAUNCHER_TARGET_VELOCITY);
            shooter2.setVelocity(LAUNCHER_TARGET_VELOCITY);
        }
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            LAUNCHER_TARGET_VELOCITY+=10;
            shooter1.setVelocity(LAUNCHER_TARGET_VELOCITY);
            shooter2.setVelocity(LAUNCHER_TARGET_VELOCITY);
        }
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            LAUNCHER_TARGET_VELOCITY-=10;
            shooter1.setVelocity(LAUNCHER_TARGET_VELOCITY);
            shooter2.setVelocity(LAUNCHER_TARGET_VELOCITY);
        }


        telemetry.addData("Target Speed", LAUNCHER_TARGET_VELOCITY);
        telemetry.addData("Shooter1 Speed", shooter1.getVelocity());
        telemetry.addData("Shooter2 Speed", shooter2.getVelocity());




        telemetry.update();
    }
}

