package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Axon Servo Test")
public class HoodOpMode extends LinearOpMode {

    private Servo servo;

    private double position = 0;

    @Override
    public void runOpMode() {

        servo = hardwareMap.get(Servo.class, "hoodServo");

        servo.setPosition(position);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                position += 0.01;
            }

            if (gamepad1.dpad_down ) {
                position -=0.01;
            }

            position = Math.max(0.0, Math.min(1.0, position));

            servo.setPosition(position);

            telemetry.addData("Position", position);
            telemetry.addData("PWM", position * 100.0 + "%");
            telemetry.update();

            sleep(20);
        }
    }
}
