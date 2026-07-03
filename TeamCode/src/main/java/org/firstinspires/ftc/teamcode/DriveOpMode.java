package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@TeleOp(name = "drive main it's ggs")
@Config
public class DriveOpMode extends CommandOpMode {

    private GamepadEx m_driver;
    private GamepadEx m_operator;

    private DriveSubsystem m_drive;
    private IntakeSubsystem m_intake;
    private ShooterSubsystem m_shooter;
    private HoodSubsystem m_hood;
    private TurretSubsystem m_turret;
    private LimelightSubsystem m_limelight;

    // FIX: was `private int shooterSpeed = -1;`. Compound assignment
    // `shooterSpeed -= 0.05` on an int implicitly narrows the double RHS back to
    // int every time -- (-1) - 0.05 = -1.05, truncated straight back to -1. This
    // control had ZERO effect; shooterSpeed could never actually change.
    private double shooterSpeed = -1;

    public static boolean fieldCentricDefault = false;
    public static double driveScale = 1.0;
    public static double rotScale = 1.0;

    private boolean m_fieldCentric = fieldCentricDefault;

    private static double squareInput(double input) {
        return Math.copySign(input * input, input);
    }

    @Override
    public void initialize() {
        m_driver = new GamepadEx(gamepad1);
        m_operator = new GamepadEx(gamepad2);

        m_hood = new HoodSubsystem(hardwareMap);

        m_drive = new DriveSubsystem(
                hardwareMap,
                new com.acmerobotics.roadrunner.Pose2d(0, 0, 0),
                telemetry
        );
        m_intake = new IntakeSubsystem(hardwareMap, telemetry);

        // FIX: m_limelight is now constructed BEFORE m_shooter needs it.
        // Previously m_shooter was built first, receiving a still-null m_limelight
        // reference that could never be replaced afterward (Java passes object
        // references by value -- reassigning the m_limelight variable later does
        // NOT retroactively update the reference already stored inside m_shooter).
        m_limelight = new LimelightSubsystem(hardwareMap, telemetry, m_drive);
        m_shooter = new ShooterSubsystem(hardwareMap, telemetry, m_intake, m_hood, m_limelight);
        m_turret = new TurretSubsystem(hardwareMap, telemetry, m_limelight);
        m_hood.setPwm(0.32);

        m_driver.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(() -> m_fieldCentric = !m_fieldCentric));

        m_driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(m_intake.runCommand());

        m_driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(m_intake.reverseCommand());
        m_operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(m_intake.runCommand());

        m_operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(m_intake.reverseCommand());

        m_operator.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(m_shooter.runAtPowerCommand(-1))
                .whenReleased(m_shooter.stop());

        m_operator.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(m_shooter.runAtPowerCommand(-shooterSpeed))
                .whenReleased(m_shooter.stop());

        m_operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(
                () -> m_hood.incrementUp()
        );
        m_operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(
                () -> m_hood.incrementDown()
        );
        m_operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whileHeld(
                () -> shooterSpeed -= 0.05
        );
        m_operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whileHeld(
                () -> shooterSpeed += 0.05
        );

        m_operator.getGamepadButton(GamepadKeys.Button.B).whileHeld(
                m_turret.lockToTarget()
        );

        // FIX: m_turret was missing from register(...). Subsystems only get their
        // periodic() called automatically by the CommandScheduler if registered --
        // this meant TurretSubsystem.periodic() (and its velRPM/posDeg/Tracking/
        // tx/power telemetry) NEVER RAN, silently, the whole time.
        register(m_drive, m_intake, m_hood, m_shooter, m_limelight, m_turret);
    }

    @Override
    public void run() {
        super.run();

        if (!isStarted() || isStopRequested()) {
            return;
        }

        double x = -squareInput(m_driver.getLeftX()) * driveScale;
        double y = squareInput(-m_driver.getLeftY()) * driveScale;
        double rot = squareInput(m_driver.getRightX()) * rotScale;
        m_drive.drive(x, y, rot, m_fieldCentric);

        // FIX: this line previously ran UNCONDITIONALLY every loop, AFTER
        // super.run() had already executed the scheduler (including whatever
        // power m_turret.lockToTarget() had just set while B was held). If the
        // operator's left stick was at rest -- which it will be if they're just
        // holding B expecting auto-tracking -- getLeftX() returns 0, and this line
        // immediately zeroed the turret power right after tracking set it. Every
        // single loop. This alone likely explains sluggish/non-responsive tracking,
        // independent of the InstantCommand and deadband bugs already fixed.
        // Now manual control only applies when B (auto-track) isn't held.
        if (!m_operator.getButton(GamepadKeys.Button.B)) {
            m_turret.setPower(m_operator.getLeftX());
        }

        Pose2d pose = m_drive.getPose();
        telemetry.addData("Drive Mode", m_fieldCentric ? "Field" : "Robot");
        telemetry.addData("tracking power", m_turret.computeTrackingPower());
        telemetry.addData("position of tag", m_limelight.getDistanceTrig()); // FIX: was calling
        // the old broken
        // getDistance(); now
        // uses the fixed trig method
        telemetry.addData("Pose X", pose.getX());
        telemetry.addData("Pose Y", pose.getY());
        telemetry.addData("Shooter speed", shooterSpeed);
        telemetry.addData("Heading Deg", pose.getHeading());
        telemetry.addData("Hood position: ", m_hood.getCurrentPwm());
        telemetry.addData("tag pos", m_limelight.getTX() + " " + m_limelight.getTY());
        telemetry.update();
    }
}