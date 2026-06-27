package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/**
 * Simple shooter test OpMode (no default command).
 *
 * Controls (gamepad2):
 *  - A: hold to run shooter at target RPM
 *  - B: press to stop shooter
 */
@TeleOp(name = "Test: Shooter", group = "Test")
@Config
public class ShooterOpMode extends CommandOpMode {

    private GamepadEx m_operator;
    private ShooterSubsystem m_shooter;

    // Target RPM for testing (tunable from Dashboard)
    public static double targetRPM = 3000.0;

    // Hard cap for safety – "maximum" shooter RPM during tests
    public static double maxRPM = 8000.0;

    @Override
    public void initialize() {
        m_operator = new GamepadEx(gamepad2);
        // Shooter without intake integration (manual only)
        m_shooter = new ShooterSubsystem(hardwareMap, telemetry);
        m_operator.getGamepadButton(GamepadKeys.Button.A).whenHeld(

                    m_shooter.runAtPowerCommand(-8000)

        );

        register(m_shooter);
    }


}

