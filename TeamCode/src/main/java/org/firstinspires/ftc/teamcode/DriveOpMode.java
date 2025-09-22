package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@TeleOp
public class DriveOpMode extends CommandOpMode {
    private GamepadEx m_driveGamepad;
    private DriveSubsystem m_driveSubsystem;

    private boolean m_fieldCentric = false;

    @Override
    public void initialize() {
        m_driveGamepad = new GamepadEx(gamepad1);
        m_driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        m_driveSubsystem.setDefaultCommand(new RunCommand(
                () -> {
                    telemetry.addData("Field-centric drive", m_fieldCentric);
//                    telemetry.update(); // NOTE: telemetry.update() seems to clear telemetry data

                    m_driveSubsystem.drive(
                            m_driveGamepad.getLeftX(),
                            m_driveGamepad.getLeftY(),
                            -m_driveGamepad.getRightX(),
                            m_fieldCentric
                    );
                }, m_driveSubsystem
        ));
        m_driveGamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(
                        () -> {
                            m_fieldCentric = !m_fieldCentric;
                        }
                ));
    }
}
