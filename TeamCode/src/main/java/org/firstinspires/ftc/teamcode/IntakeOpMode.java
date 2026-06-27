package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/**
 * Simple intake test OpMode (no default command).
 *
 * Controls (gamepad1):
 *  - Right bumper: intake in
 *  - Left bumper:  reverse / out
 *  - Neither:      stop
 */
@TeleOp(name = "Test: Intake", group = "Test")
@Config
public class IntakeOpMode extends CommandOpMode {

    private GamepadEx m_driver;
    private IntakeSubsystem m_intake;
    private ShooterSubsystem m_shooter;

    @Override
    public void initialize() {
        m_driver = new GamepadEx(gamepad1);
        m_intake = new IntakeSubsystem(hardwareMap, telemetry);
        m_shooter = new ShooterSubsystem(hardwareMap, telemetry);

        m_driver.getGamepadButton(GamepadKeys.Button.A).whenHeld(
                new ParallelCommandGroup(
                        m_intake.runCommand(), m_shooter.runAtVelocityCommand(-8000)
                )
        );
        telemetry.addData("Intake Power", m_intake.getPower());
        telemetry.update();

        // Register intake so periodic() runs
        register(m_intake);
    }


}