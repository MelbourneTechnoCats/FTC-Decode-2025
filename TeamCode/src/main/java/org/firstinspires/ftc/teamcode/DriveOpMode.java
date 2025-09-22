package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@TeleOp
public class DriveOpMode extends CommandOpMode {
    private GamepadEx m_driveGamepad;
    private DriveSubsystem m_driveSubsystem;

    @Override
    public void initialize() {
        m_driveGamepad = new GamepadEx(gamepad1);
        m_driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        m_driveSubsystem.setDefaultCommand(new RunCommand(
                () -> {
                    m_driveSubsystem.drive(
                            m_driveGamepad.getLeftX(),
                            m_driveGamepad.getLeftY(),
                            -m_driveGamepad.getRightX(),
                            false
                    );
                }, m_driveSubsystem
        ));
    }
}
