package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

/**
 * Main TeleOp mode integrating:
 * - Drive + Vision
 * - Turret (tx-based tracking)
 * - Shooter + Hood (with spline interpolation)
 * - Intake (single motor with pre-shoot reverse)
 */
@TeleOp(name = "Drive + Shooter + Turret", group = "TeleOp")
@Config
public class DriveOpMode extends CommandOpMode {

    // Gamepads
    private GamepadEx m_driver;
    private GamepadEx m_operator;

    // Subsystems
    private DriveSubsystem m_drive;
    private IntakeSubsystem m_intake;
    private ShooterSubsystem m_shooter;
//    private VisionSubsystem m_vision;
    private TurretSubsystem m_turret;
//    private HoodSubsystem m_hood;

    // Drive settings
    public static boolean fieldCentricDefault = false;
    public static double driveScale = 1.0;
    public static double rotScale = 1.0;

    private boolean m_fieldCentric = fieldCentricDefault;

    public static double squareInput(double input) {
        return Math.copySign(input * input, input);
    }

    @Override
    public void initialize() {
        m_driver = new GamepadEx(gamepad1);
        m_operator = new GamepadEx(gamepad2);


        // Vision (used by Drive and Turret)
//        m_vision = new VisionSubsystem(hardwareMap, telemetry);

        // Drive
        m_drive = new DriveSubsystem(hardwareMap, new com.acmerobotics.roadrunner.Pose2d(0, 0, 0), telemetry);

        // Intake (single motor)
        m_intake = new IntakeSubsystem(hardwareMap, telemetry);

        // Shooter + Hood (now passes intake for pre-shoot behavior)
//        m_shooter = new ShooterSubsystem(hardwareMap, telemetry, m_intake);
//        m_hood = m_shooter.getHood();

        // Turret
        m_turret = new TurretSubsystem(hardwareMap, telemetry);

        // ==================== Default Commands ====================

//        // Default Drive Command
//        m_drive.setDefaultCommand(
//            new RunCommand(() -> {
//                double lx = m_driver.getLeftX();
//                double ly = m_driver.getLeftY();
//                double rx = m_driver.getRightX();
//
//                double x = squareInput(lx) * driveScale;
//                double y = squareInput(ly) * driveScale;
//                double rot = -squareInput(rx) * rotScale;
//
//                m_drive.drive(x, y, rot, m_fieldCentric);
//
//                Pose2d pose = m_drive.getPose();
//                telemetry.addData("Field Centric", m_fieldCentric);
//                telemetry.addData("X", pose.getX());
//                telemetry.addData("Y", pose.getY());
//                telemetry.addData("Heading (deg)", pose.getHeading());
//            }, m_drive)
//        );

        // Default Turret - manual control with right stick
        

        // ==================== Driver Controls ====================

//        // Toggle Field Centric
//        m_driver.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
//                .whenPressed(new InstantCommand(() -> m_fieldCentric = !m_fieldCentric));
//
//        // Intake Controls
//        m_driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                .whileHeld(m_intake.runCommand());
//
//        m_driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                .whileHeld(m_intake.reverseCommand());

        // ==================== Operator Controls ====================

        // Shoot using vision range (Blue / Red)
        // This will trigger intake pre-shoot sequence (reverse → forward) + shooter + hood
        /*m_operator.getGamepadButton(GamepadKeys.Button.A)
                .whileHeld(() -> {
                    double range = m_vision.getBlueTargetRange();
                    if (!Double.isNaN(range)) {
                        schedule(m_shooter.shootAtDistance(range));
                    }
                });

        m_operator.getGamepadButton(GamepadKeys.Button.B)
                .whileHeld(() -> {
                    double range = m_vision.getRedTargetRange();
                    if (!Double.isNaN(range)) {
                        schedule(m_shooter.shootAtDistance(range));
                    }
                });*/

        // Manual Hood Nudge
//        m_operator.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                .whenPressed(new InstantCommand(m_hood::incrementUp));
//
//        m_operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                .whenPressed(new InstantCommand(m_hood::incrementDown));

        // Turret Controls
//        m_operator.getGamepadButton(GamepadKeys.Button.X)
//                .whenPressed(m_turret.autoTrackWithScanCommand());
//
//        m_operator.getGamepadButton(GamepadKeys.Button.Y)
//                .whenPressed(m_turret.stopCommand());

        // Register all subsystems
        register(m_drive, m_intake, m_shooter, m_turret);
    }
}