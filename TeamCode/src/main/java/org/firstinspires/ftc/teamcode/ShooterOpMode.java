package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.IntakeAndSorterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SorterSubsystem;

@TeleOp
@Config
public class ShooterOpMode extends CommandOpMode {
    private GamepadEx m_shootGamepad;
    private ShooterSubsystem m_shooterSubsystem;
    private SorterSubsystem m_sorterSubsystem;
    private IntakeSubsystem m_intakeSubsystem;
    private IntakeAndSorterSubsystem m_intakeAndSorter;

    public static double m_velocity = 1800;

    @Override
    public void initialize() {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        m_shootGamepad = new GamepadEx(gamepad1);
        m_sorterSubsystem = new SorterSubsystem(hardwareMap);
        m_intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        m_intakeAndSorter = new IntakeAndSorterSubsystem(m_intakeSubsystem, m_sorterSubsystem);
        m_shooterSubsystem = new ShooterSubsystem(hardwareMap, m_intakeAndSorter, telemetry);

        m_shootGamepad.getGamepadButton(GamepadKeys.Button.A) // use this for calibrating servo
                .whenHeld(
                        new SelectCommand(() -> m_shooterSubsystem.runCommand(0, m_velocity))
                );

        m_shootGamepad.getGamepadButton(GamepadKeys.Button.B) // typical shooting
                .whenHeld(
                        new SelectCommand(() -> m_shooterSubsystem.runCommand(60, m_velocity))
                );

        m_shootGamepad.getGamepadButton(GamepadKeys.Button.Y) // should be maximum range
                .whenHeld(
                        new SelectCommand(() -> m_shooterSubsystem.runCommand(90, m_velocity))
                );

        m_shootGamepad.getGamepadButton(GamepadKeys.Button.X) // load ball in and shoot
                .whenPressed(
                        new SelectCommand(() -> {
                            for (int i = 0; i < 3; i++) m_sorterSubsystem.occupancy[i] = SorterSubsystem.Colour.GREEN;
                            return m_shooterSubsystem.shootCommandWithVelocity(m_velocity, 60);
                        })
                );

        m_intakeAndSorter.setDefaultCommand(new RunCommand(() -> { telemetry.update(); }, m_intakeAndSorter));

        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        schedule(new RunCommand(() -> {
            dashboardTelemetry.addData("left vel", m_shooterSubsystem.getLeftVelocity());
            dashboardTelemetry.addData("right vel", m_shooterSubsystem.getRightVelocity());
            dashboardTelemetry.update();
        }));
    }
}
