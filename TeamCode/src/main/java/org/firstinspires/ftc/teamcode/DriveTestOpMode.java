//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.seattlesolvers.solverslib.command.CommandOpMode;
//import com.seattlesolvers.solverslib.command.RunCommand;
//import com.seattlesolvers.solverslib.gamepad.GamepadEx;
//import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
//
///**
// * Main TeleOp:
// *  - gamepad1: drive
// *  - gamepad1 RB/LB: intake in / out
// *  - gamepad2 A: shooter run at RPM
// */
//@TeleOp
//@Config
//public class DriveTestOpMode extends CommandOpMode {
//
//    private GamepadEx m_driver;
//    private GamepadEx m_operator;
//
//    private DriveSubsystem m_drive;
//
//    private IntakeSubsystem m_intake;
//    private ShooterSubsystem m_shooter;
//
//    public static boolean fieldCentricDefault = false;
//    public static double driveScale = 1.0;
//    public static double rotScale = 1.0;
//
//    public static double shooterTestRPM = 5000.0;
//
//    private boolean m_fieldCentric = fieldCentricDefault;
//
//    private static double squareInput(double input) {
//        return Math.copySign(input * input, input);
//    }
//
//    @Override
//    public void initialize() {
//        m_driver   = new GamepadEx(gamepad1);
//        m_operator = new GamepadEx(gamepad2);
//        HoodSubsystem m_hood = new HoodSubsystem(hardwareMap);
//        m_drive   = new DriveSubsystem(hardwareMap,
//                new com.acmerobotics.roadrunner.Pose2d(0, 0, 0), telemetry);
//        m_intake  = new IntakeSubsystem(hardwareMap, telemetry);
//        m_shooter = new ShooterSubsystem(hardwareMap, telemetry, m_intake, m_hood);
//
//        // ---------- Default Drive Command ----------
//        m_drive.setDefaultCommand(
//                new RunCommand(() -> {
//                    double lx = m_driver.getLeftX();
//                    double ly = m_driver.getLeftY();
//                    double rx = m_driver.getRightX();
//
//                    double x   = squareInput(lx) * driveScale;
//                    double y   = squareInput(ly) * driveScale;
//                    double rot = -squareInput(rx) * rotScale;
//
//                    m_drive.drive(x, y, rot, m_fieldCentric);
//
//                    com.seattlesolvers.solverslib.geometry.Pose2d pose = m_drive.getPose();
//                    telemetry.addData("FieldCentric", m_fieldCentric);
//                    telemetry.addData("X", pose.getX());
//                    telemetry.addData("Y", pose.getY());
//                    telemetry.addData("Heading (deg)", pose.getHeading());
//                    telemetry.update();
//                }, m_drive)
//        );
//
//        // ---------- Driver controls ----------
//        // Toggle field-centric
//        m_driver.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
//                .whenPressed(() -> m_fieldCentric = !m_fieldCentric);
//
//        // Intake in / out
//        m_driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                .whileHeld(m_intake.runCommand());
//
//        m_driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                .whileHeld(m_intake.reverseCommand());
//
//        // ---------- Operator controls ----------
//        // Shooter at fixed RPM while A held
//        m_operator.getGamepadButton(GamepadKeys.Button.A)
//                .whileHeld(m_shooter.runAtVelocityCommand(shooterTestRPM));
//
//        m_operator.getGamepadButton(GamepadKeys.Button.B)
//                .whenPressed(m_shooter.stop());
//
//        // Register subsystems so periodic() runs
//        register(m_drive, m_intake, m_shooter);
//    }
//}
