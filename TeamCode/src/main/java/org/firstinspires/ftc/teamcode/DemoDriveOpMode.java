package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MotorSubsystem;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "we are so so cooked", group = "event opmode" +
        "" +
        "")
@Config
public class DemoDriveOpMode extends CommandOpMode {

    private GamepadEx m_gamepad;

    private MotorSubsystem m_frontLeft;
    private MotorSubsystem m_frontRight;
    private MotorSubsystem m_backLeft;
    private MotorSubsystem m_backRight;

    private MotorSubsystem m_intakeMotor;
    // Mirrored shooter motors
    private MotorSubsystem m_shooterLeft;
    private MotorSubsystem m_shooterRight;

    public static String frontLeftName  = "frontLeftDrive";
    public static String frontRightName = "frontRightDrive";
    public static String backLeftName   = "backLeftDrive";
    public static String backRightName  = "backRightDrive";

    public static String intakeMotorName   = "intakeMotor";
    public static String shooterLeftName   = "shooterMotor";


    public static double encoderCpr = 28.0;
    public static double driveScale = 1.0;

    // Everything set to full power
    public static double intakeInPower   = 1.0;
    public static double intakeOutPower  = -1.0;
    public static double shooterPower    = 1;

    
    public static long intakeBeforeShootMs = 2050;

    // timestamp for shooter sequence
    private double shooterStartTime = 0.0;

    @Override
    public void initialize() {
        m_gamepad = new GamepadEx(gamepad1);

        m_frontLeft  = new MotorSubsystem(hardwareMap, frontLeftName,  encoderCpr, true);
        m_backLeft   = new MotorSubsystem(hardwareMap, backLeftName,   encoderCpr, true);
        m_frontRight = new MotorSubsystem(hardwareMap, frontRightName, encoderCpr, false);
        m_backRight  = new MotorSubsystem(hardwareMap, backRightName,  encoderCpr, false);

        m_intakeMotor  = new MotorSubsystem(hardwareMap, intakeMotorName,  encoderCpr, false);

        // Initialize mirrored shooter motors
        m_shooterLeft  = new MotorSubsystem(hardwareMap, shooterLeftName,  encoderCpr, false);
        // One is usually reversed
        
        m_frontLeft.setDefaultCommand(new RunCommand(() -> {
            // UNMIRRORED: Standard stick inputs
            double x = -squareInput(m_gamepad.getLeftX());
            double y = squareInput(m_gamepad.getLeftY());
            double rot = squareInput(m_gamepad.getRightX());

            // Standard Mecanum Power Mix
            double fl = y + x + rot;
            double fr = y - x - rot;
            double bl = y - x + rot;
            double br = y + x - rot;

            double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
            if (max > 1.0) {
                fl /= max; fr /= max; bl /= max; br /= max;
            }

            m_frontLeft.setRawPower(-fl * driveScale);
            m_frontRight.setRawPower(-fr * driveScale);
            m_backLeft.setRawPower(-bl * driveScale);
            m_backRight.setRawPower(-br * driveScale);

            // --- Button Debug Telemetry ---
            List<String> pressedButtons = new ArrayList<>();
            for (GamepadKeys.Button button : GamepadKeys.Button.values()) {
                if (m_gamepad.getButton(button)) {
                    pressedButtons.add(button.toString());
                }
            }
            telemetry.addData("Buttons Pressed", pressedButtons.isEmpty() ? "None" : pressedButtons.toString());
            telemetry.addData("Power of shooter motor:", shooterPower);
            telemetry.update();
        }, m_frontLeft, m_frontRight, m_backLeft, m_backRight));

        // --- Intake controls (Full Power) ---
        m_gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(m_intakeMotor.setPowerCommand(intakeInPower));

        m_gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(m_intakeMotor.setPowerCommand(intakeOutPower));
        m_gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                        .whenPressed(new InstantCommand( () -> {
                             shooterPower+=0.05;
                        }));
        m_gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed( new InstantCommand( () -> {
                    if (shooterPower>0) shooterPower -=0.05;
                }));


        // --- Optimized Shooter sequence (Full Power & Mirrored) ---
         m_gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(
                        new InstantCommand(() -> {
                            shooterStartTime = getRuntime();
                        })
                )
                .whileHeld(
                        new RunCommand(() -> {
                            double elapsedMs = (getRuntime() - shooterStartTime) * 1000.0;

                            if (elapsedMs < intakeBeforeShootMs) {
                                // Phase 1: intake only
                                m_intakeMotor.setRawPower(0);
                                m_shooterLeft.setRawPower(-shooterPower);
                            } else {
                                // Phase 2: intake + shooter together
                                m_intakeMotor.setRawPower(1);
                                m_shooterLeft.setRawPower(-shooterPower);
                            }
                        }, m_intakeMotor, m_shooterLeft)
                )
                .whenReleased(
                        new InstantCommand(() -> {
                            // Stop intake and shooter when A is released
                            m_intakeMotor.setRawPower(0.0);
                            m_shooterLeft.setRawPower(0.0);
                        })
                );

        // Stop shooter when A is released
        
    }

    public static double squareInput(double input) {
        return Math.copySign(input * input, input);
    }
}