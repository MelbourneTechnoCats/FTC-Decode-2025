package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.SorterSubsystem;

@TeleOp
public class SorterOpMode extends CommandOpMode {
    private SorterSubsystem m_sorterSubsystem;
    private GamepadEx m_gamepad;
    private boolean m_toIntake = true;
    private boolean m_retract = true;
    private int m_position = 0;

    @Override
    public void initialize() {
        m_gamepad = new GamepadEx(gamepad1);
        m_sorterSubsystem = new SorterSubsystem(hardwareMap);
        m_gamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(
                        new InstantCommand(() -> {
                            m_toIntake = !m_toIntake;
                            telemetry.addData("Intake", m_toIntake);
                            telemetry.addData("Position", m_position);
                            telemetry.update();
                        })
                                .andThen(new SelectCommand(() -> m_sorterSubsystem.setSorterAngleCommand(m_position, m_toIntake)))
                );
        m_gamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(
                        new InstantCommand(() -> {
                            m_position++;
                            if (m_position > 2)
                                m_position = 0;
                            telemetry.addData("Intake", m_toIntake);
                            telemetry.addData("Position", m_position);
                            telemetry.update();
                        })
                                .andThen(new SelectCommand(() -> m_sorterSubsystem.setSorterAngleCommand(m_position, m_toIntake)))
                );
        m_gamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(
                        new InstantCommand(() -> {
                            m_retract = !m_retract;
                            telemetry.addData("Retract", m_retract);
                            telemetry.update();
                        })
                                .andThen(new SelectCommand(() -> m_sorterSubsystem.setLeverAngleCommand(m_retract)))
                );
    }
}
