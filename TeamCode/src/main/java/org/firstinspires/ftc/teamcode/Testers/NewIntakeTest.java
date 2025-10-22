package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.V1.Config.subsystems.Intake;

@Disabled
@TeleOp
public class NewIntakeTest extends OpMode{

        private GamepadEx player1;
        private Intake intake;
        private double increment = 50;
        private double startingTargetRPM = 800;

        @Override
        public void init(){
            player1 = new GamepadEx(gamepad1);
            intake = new Intake(hardwareMap, telemetry);

            telemetry.addLine("Ready!");
            telemetry.update();
        }

        @Override
        public void loop() {
            player1.readButtons();

            if (player1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                intake.incrementRPM(increment);
            }
            else if (player1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                intake.decrementRPM(increment);
            }

            if (player1.wasJustPressed((GamepadKeys.Button.SQUARE))) {
                intake.stopIntake();

            } else if (player1.wasJustPressed((GamepadKeys.Button.CIRCLE))) {
               intake.setTargetRPM(startingTargetRPM);
            }

            if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) {
                intake.toggleServoDirection();
            }

            while (player1.isDown(GamepadKeys.Button.X)) {
                intake.setServoPower(0.5);
            }



            // Convert RPM to ticks per second

            telemetry.addData("Target RPM", intake.getCurrentTargetRPM());
            telemetry.addData("Intake RPM", intake.getCurrentRPM());

            telemetry.addData("Target TPS", intake.getCurrentTargetVelocity());
            telemetry.addData("Intake TPS", intake.getCurrentVelocity());

            telemetry.addData("Servo Power", intake.getServoPower()); // servoIntake.getPower()

            telemetry.update();
        }

    }


