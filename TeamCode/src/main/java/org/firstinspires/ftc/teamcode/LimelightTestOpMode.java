package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/**
 * Limelight/AprilTag heading alignment test.
 *
 * - Left stick: drive (x/y).
 * - Right stick X: normal rotation.
 * - Hold A: Auto-align robot heading using direct tx offset from Limelight.
 * - Uses Limelight Pipeline 1 directly.
 */
@TeleOp(name = "Test: Limelight Heading Align", group = "Test")
@Config
public class LimelightTestOpMode extends CommandOpMode {

    // --- Motor configuration ---
    public static String frontLeftName  = "frontLeftDrive";
    public static String frontRightName = "frontRightDrive";
    public static String backLeftName   = "backLeftDrive";
    public static String backRightName  = "backRightDrive";

    public static double encoderCpr = 28.0;
    public static double driveScale = 1.0;

    // --- Alignment tuning ---
    public static boolean kUseBlueTag    = true;
    public static double kP = 0.035; // Power per degree of tx error
    public static double kMaxTurnPower   = 0.5;
    public static double kHeadingTolDeg  = 0.2;

    private GamepadEx m_gamepad;
    private LimelightSubsystem m_limelight;

    private MotorSubsystem m_frontLeft;
    private MotorSubsystem m_frontRight;
    private MotorSubsystem m_backLeft;
    private MotorSubsystem m_backRight;

    private ShooterSubsystem m_shooter;

    // Variable to track heading
    private double m_robotHeading = 0.0;

    @Override
    public void initialize() {
        m_gamepad = new GamepadEx(gamepad1);

        // Subsystems
        m_limelight = new LimelightSubsystem(hardwareMap, telemetry, null);

        // Drivetrain motors
        m_frontLeft  = new MotorSubsystem(hardwareMap, frontLeftName,  encoderCpr, true);
        m_backLeft   = new MotorSubsystem(hardwareMap, backLeftName,   encoderCpr, true);
        m_frontRight = new MotorSubsystem(hardwareMap, frontRightName, encoderCpr, false);
        m_backRight  = new MotorSubsystem(hardwareMap, backRightName,  encoderCpr, false);

        // Default command: normal mecanum drive
        m_frontLeft.setDefaultCommand(new RunCommand(() -> {
            double x   =  DemoDriveOpMode.squareInput(m_gamepad.getLeftX());
            double y   = -DemoDriveOpMode.squareInput(m_gamepad.getLeftY());
            double rot = -DemoDriveOpMode.squareInput(m_gamepad.getRightX());

            setDrivePower(x, y, rot);
            String targetPoseCamera = "";
            if (!m_limelight.limelight.getLatestResult().getFiducialResults().isEmpty()) targetPoseCamera = m_limelight.limelight.getLatestResult().getFiducialResults().get(0).getTargetPoseCameraSpace().getPosition().toString();
                    else targetPoseCamera = "no target";




            String targetPoseRobot = "";
            if (!m_limelight.limelight.getLatestResult().getFiducialResults().isEmpty()) targetPoseRobot = m_limelight.limelight.getLatestResult().getFiducialResults().get(0).getTargetPoseRobotSpace().getPosition().toString();
            else targetPoseRobot = "no target";

            // Update heading variable
            m_robotHeading = m_limelight.getRobotHeading();

            // Telemetry
            telemetry.addData("Robot Heading (deg)", "%.2f", m_robotHeading);
            telemetry.addData("Limelight tx", "%.2f", m_limelight.getTX());
            telemetry.addData("Has Tag", m_limelight.hasTarget());
            telemetry.addData("position according to camera", targetPoseCamera);
            telemetry.addData("position according to robot", targetPoseRobot);
            telemetry.update();
        }, m_frontLeft, m_frontRight, m_backLeft, m_backRight));

        // A button: auto-align once when pressed
        m_gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(alignToTagDirectCommand());
    }

    /**
     * Direct alignment command using Limelight tx offset.
     * Ends automatically when tx is within tolerance.
     */
    private Command alignToTagDirectCommand() {
        return new FunctionalCommand(
                () -> {}, // init
                () -> {
                    if (!m_limelight.hasTarget()) {
                        setDrivePower(0, 0, 0);
                        telemetry.addData("Align", "No target");
                        return;
                    }

                    double tx = m_limelight.getTX();

                    // Rotate clockwise (positive) if tx > 0, anti-clockwise (negative) if tx < 0
                    double output = tx * kP;
                    output = Math.max(-kMaxTurnPower, Math.min(kMaxTurnPower, output));

                    setDrivePower(0, 0, output);
                    telemetry.addData("Align", "Active");
                    telemetry.addData("tx Error (deg)", "%.2f", tx);
                },
                interrupted -> setDrivePower(0, 0, 0),
                () -> !m_limelight.hasTarget() || Math.abs(m_limelight.getTX()) < kHeadingTolDeg,
                m_frontLeft, m_frontRight, m_backLeft, m_backRight
        );
    }

    /**
     * Mecanum mixing helper.
     */
    private void setDrivePower(double x, double y, double rot) {
        double flRaw = y + x + rot;
        double frRaw = y - x - rot;
        double blRaw = y - x + rot;
        double brRaw = y + x - rot;

        double max = Math.max(Math.max(Math.abs(flRaw), Math.abs(frRaw)),
                              Math.max(Math.abs(blRaw), Math.abs(brRaw)));
        
        double fl = max > 1.0 ? flRaw / max : flRaw;
        double fr = max > 1.0 ? frRaw / max : frRaw;
        double bl = max > 1.0 ? blRaw / max : blRaw;
        double br = max > 1.0 ? brRaw / max : brRaw;

        m_frontLeft.setRawPower(fl * driveScale);
        m_frontRight.setRawPower(fr * driveScale);
        m_backLeft.setRawPower(bl * driveScale);
        m_backRight.setRawPower(br * driveScale);
    }
}
