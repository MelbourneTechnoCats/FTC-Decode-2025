//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.RunCommand;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.subsystems.NoOdometryDriveSubsystem;
//
///**
// * Simple TeleOp to test NoOdometryDriveSubsystem.
// *
// * - Left stick: translation (x/y).
// * - Right stick X: rotation.
// * - Left stick button: toggle field‑centric.
// * - Telemetry: heading + commanded field velocity.
// */
//@TeleOp(name = "Test: No-Odo Drive", group = "Test")
//@Config
//public class DriveTestOpMode extends CommandOpMode {
//
//    private GamepadEx m_gamepad;
//    private NoOdometryDriveSubsystem m_drive;
//    private boolean m_fieldCentric = false;
//
//    public static double driveScale = 1.0;
//    public static double rotScale   = 1.0;
//
//    private static double squareInput(double input) {
//        return Math.copySign(input * input, input);
//    }
//
//    @Override
//    public void initialize() {
//        m_gamepad = new GamepadEx(gamepad1);
//        m_drive   = new NoOdometryDriveSubsystem(hardwareMap, telemetry);
//
//        // Default driving command
//        m_drive.setDefaultCommand(
//                new RunCommand(() -> {
//                    double leftX  = m_gamepad.getLeftX();
//                    double leftY  = m_gamepad.getLeftY();
//                    double rightX = m_gamepad.getRightX();
//
//                    double x = squareInput(leftX) * driveScale;
//                    double y = squareInput(leftY) * driveScale;
//                    double rot = -squareInput(rightX) * rotScale;
//
//                    m_drive.drive(x, y, rot, m_fieldCentric);
//
//                    telemetry.addData("Field-centric", m_fieldCentric);
//                    telemetry.addData("Heading (deg)", m_drive.getHeading().getDegrees());
//                    telemetry.addData("Cmd Field Vel X", m_drive.getFieldVelocity().getX());
//                    telemetry.addData("Cmd Field Vel Y", m_drive.getFieldVelocity().getY());
//                }, m_drive)
//        );
//
//        // Toggle field-centric on left stick button
//        m_gamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
//                .whenPressed(() -> m_fieldCentric = !m_fieldCentric);
//    }
//}