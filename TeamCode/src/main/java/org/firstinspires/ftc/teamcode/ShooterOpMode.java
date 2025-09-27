package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
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
        m_shooterSubsystem = new ShooterSubsystem(hardwareMap);

        m_shootGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(
                        () -> {
                            m_shooterSubsystem.setPower(1);
                        }
                )).whenReleased(
                        new InstantCommand(
                                () -> {
                                    m_shooterSubsystem.stop();
                                }
                        )
                );
        m_shootGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(
                        () -> {
                            m_shooterSubsystem.setAngle(180);
                        }
                ));
        m_shootGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(
                        new InstantCommand(
                                () -> {
                                    m_shooterSubsystem.turnByAngle(-1);

                                }
                        )
                );
        m_shootGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(
                        new InstantCommand(
                                () -> {
                                    m_shooterSubsystem.turnByAngle(1);
                                }
                        )
                );

    }
}
