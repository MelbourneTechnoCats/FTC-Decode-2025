package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SorterSubsystem;

@TeleOp
@Config
public class ShooterOpMode extends CommandOpMode {
    private GamepadEx m_shootGamepad;
    private ShooterSubsystem m_shooterSubsystem;
    private SorterSubsystem m_sorterSubsystem;

    public static double m_velocity = 1800;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        m_shootGamepad = new GamepadEx(gamepad1);
        m_sorterSubsystem = new SorterSubsystem(hardwareMap);
        m_shooterSubsystem = new ShooterSubsystem(hardwareMap, m_sorterSubsystem, telemetry);

        m_shootGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whileHeld(new RunCommand(() -> {
                    m_shooterSubsystem.setVelocity(m_velocity);
                }, m_shooterSubsystem)).whenReleased(
                        m_shooterSubsystem.stopCommand()
                );
        m_shootGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(
                        new InstantCommand(
                                () -> {
                                    m_shooterSubsystem.turnByAngle(-1);

                                }, m_shooterSubsystem
                        )
                );
        m_shootGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(
                        new InstantCommand(
                                () -> {
                                    m_shooterSubsystem.turnByAngle(1);
                                }, m_shooterSubsystem
                        )
                );

    }
}
