package org.firstinspires.ftc.teamcode.MotorTest;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MotorTest.Subsystems.MotorSubsystem;

@TeleOp
public class MotorTestOpMode extends CommandOpMode {
    private MotorSubsystem _motor;
    private GamepadEx _gamepad;

    @Override
    public void initialize() {
        _gamepad = new GamepadEx(gamepad1);

        _motor = new MotorSubsystem(hardwareMap.get(DcMotor.class, "motor"), telemetry);

        _gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whileHeld(new RunCommand(() -> {

                    _motor.runMotor(1);
                }).whenFinished(_motor::stopMotor));
    }
}
















