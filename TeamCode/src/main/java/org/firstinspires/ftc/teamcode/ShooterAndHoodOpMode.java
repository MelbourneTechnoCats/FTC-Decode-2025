package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/**
 * Combined Shooter + Hood test OpMode.
 *
 * Controls:
 *  gamepad2:
 *    - Right trigger: shooter power (0..shooterPowerScale) forward
 *    - Left  trigger: shooter power (0..shooterPowerScale) reverse
 *    - Dpad up/down:  nudge hood PWM up/down by STEP_POS
 *
 *  Telemetry:
 *    - Shooter target & actual RPM
 *    - Hood PWM position
 */
@TeleOp(name = "Test: Shooter + Hood", group = "Test")
@Config
public class ShooterAndHoodOpMode extends CommandOpMode {

    private GamepadEx m_operator;
    private ShooterSubsystem m_shooter;
    private HoodSubsystem m_hood;

    // Max open-loop shooter power for triggers
    public static double shooterPowerScale = 1.0;

    // Extra hood nudge scale if you want it different from STEP_POS
    public static double hoodStepOverride = 0.0; // 0 = use HoodSubsystem.STEP_POS

    private static double squareInput(double input) {
        return Math.copySign(input * input, input);
    }

    @Override
    public void initialize() {
        m_operator = new GamepadEx(gamepad2);

        // Standalone shooter + hood (no intake/drive needed for this test)
        m_hood = new HoodSubsystem(hardwareMap);
        m_shooter = new ShooterSubsystem(hardwareMap, telemetry);




        m_operator.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whileHeld(() -> {
                    m_hood.incrementUp();
                });

        // Dpad Down: nudge hood down
        m_operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whileHeld(() -> {
                    m_hood.incrementDown();
                });
        telemetry.addData("position", m_hood.getCurrentPwm());
        m_operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                m_shooter.runAtPowerCommand(-100000)
        );

        register(m_shooter, m_hood);
    }
}
