package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MotorSubsystem;

@TeleOp(name = "Test: Motor Speeds", group = "Test")
@Config
public class MotorTestOpMode extends CommandOpMode {

    private GamepadEx m_gamepad;
    private MotorSubsystem m_motor;

    // Change this to the config name of the motor you want to test
    public static String motorName = "frontLeftDrive";

    // Encoder CPR for your motor (e.g. 28 for goBILDA 5202/5203/5204)
    public static double encoderCpr = 28.0;

    // List of test powers (you can tune from Dashboard)
    public static double[] testPowers = {0.2, 0.4, 0.6, 0.8, 1.0};

    // Index into testPowers
    public static int powerIndex = 0;

    @Override
    public void initialize() {
        m_gamepad = new GamepadEx(gamepad1);

        // Plain MotorSubsystem: open-loop power control
        m_motor = new MotorSubsystem(hardwareMap, motorName, encoderCpr);

        // Telemetry loop
        m_motor.setDefaultCommand(new RunCommand(() -> {
            double velRpm = m_motor.getVelocity();
            double currentPower = getCurrentPower();

            telemetry.addData("Motor", motorName);
            telemetry.addData("Power index", powerIndex);
            telemetry.addData("Current power", currentPower);
            telemetry.addData("Velocity (RPM)", velRpm);
            addButtonTelemetry();
            telemetry.update();
        }, m_motor));

        // D-pad up/down to change power index
        m_gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> {
                    powerIndex++;
                    if (powerIndex >= testPowers.length) powerIndex = testPowers.length - 1;
                });
        m_gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> {
                    powerIndex--;
                    if (powerIndex < 0) powerIndex = 0;
                });

        // Hold A to run motor at selected power, release to stop
        m_gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whileHeld(m_motor.setPowerCommand(this::getCurrentPower));
        
    }

    private double getCurrentPower() {
        if (testPowers == null || testPowers.length == 0) return 0.0;
        int idx = Math.max(0, Math.min(powerIndex, testPowers.length - 1));
        return testPowers[idx];
    }
    private void addButtonTelemetry() {
        if (m_gamepad == null) return;

        StringBuilder pressed = new StringBuilder();

        if (m_gamepad.getButton(GamepadKeys.Button.A)) pressed.append("A ");
        if (m_gamepad.getButton(GamepadKeys.Button.B)) pressed.append("B ");
        if (m_gamepad.getButton(GamepadKeys.Button.X)) pressed.append("X ");
        if (m_gamepad.getButton(GamepadKeys.Button.Y)) pressed.append("Y ");

        if (m_gamepad.getButton(GamepadKeys.Button.DPAD_UP)) pressed.append("DUP ");
        if (m_gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) pressed.append("DDOWN ");
        if (m_gamepad.getButton(GamepadKeys.Button.DPAD_LEFT)) pressed.append("DLEFT ");
        if (m_gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)) pressed.append("DRIGHT ");

        if (m_gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) pressed.append("LB ");
        if (m_gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) pressed.append("RB ");

        if (m_gamepad.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)) pressed.append("LS ");
        if (m_gamepad.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)) pressed.append("RS ");

        double lx = m_gamepad.getLeftX();
        double ly = m_gamepad.getLeftY();
        double rx = m_gamepad.getRightX();
        double ry = m_gamepad.getRightY();

        telemetry.addData("Buttons pressed", pressed.length() == 0 ? "none" : pressed.toString());
        telemetry.addData("Left stick", "x=%.2f y=%.2f", lx, ly);
        telemetry.addData("Right stick", "x=%.2f y=%.2f", rx, ry);
    }
}