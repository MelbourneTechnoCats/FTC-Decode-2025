package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@TeleOp
public class ShooterOpMode extends CommandOpMode {
    private GamepadEx m_shootGamepad;
    private ShooterSubsystem m_shooterSubsystem;

    @Override
    public void initialize() {
        m_shootGamepad = new GamepadEx(gamepad1);
        m_shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetry);

        m_shootGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(m_shooterSubsystem.runCommand(180, 1000)).whenReleased(
                        m_shooterSubsystem.stopCommand()
                );
        m_shootGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        () -> { ShooterSubsystem.kshooterS += 0.1; }
                );
        m_shootGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        () -> { ShooterSubsystem.kshooterS -= 0.1; }
                );
        m_shootGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(
                        () -> { ShooterSubsystem.kshooterV += 0.1; }
                );
        m_shootGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(
                        () -> { ShooterSubsystem.kshooterV -= 0.1; }
                );
    }
}
