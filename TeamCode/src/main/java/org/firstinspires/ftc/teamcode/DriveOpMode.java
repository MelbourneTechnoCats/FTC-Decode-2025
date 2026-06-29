package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Teleop
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
        m_shooter = new ShooterSubsystem(hardwareMap, telemetry, m_intake, m_hood, m_limelight);
        m_limelight = new LimelightSubsystem(hardwareMap, telemetry, m_drive);
        m_turret = new TurretSubsystem(hardwareMap, telemetry, m_limelight);

//

        m_driver.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(() -> m_fieldCentric = !m_fieldCentric));

        m_driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(m_intake.runCommand());

        m_driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(m_intake.reverseCommand());

        m_driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ParallelCommandGroup(
                        m_intake.runCommand(), m_shooter.runAtPowerCommand(-1)
                ))
                .whenReleased(
                        m_shooter.stop()
                );
        m_driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(
                () ->  m_hood.incrementUp()
        );
        m_driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(
                () -> m_hood.incrementDown()
        );
        m_operator.getGamepadButton(GamepadKeys.Button.A).whileHeld(
                m_turret.lockToTarget()
        );


        register(m_drive, m_intake);
    }
    @Override
    public void run() {
        super.run();   // let CommandOpMode scheduler run buttons, etc.

        if (!isStarted() || isStopRequested()) {
            return;
        }

        double x   = squareInput(m_driver.getLeftX()) * driveScale;
        double y   = squareInput(-m_driver.getLeftY()) * driveScale;
        double rot = squareInput(m_driver.getRightX()) * rotScale;
        m_drive.drive(x, y, rot, m_fieldCentric);

        

        Pose2d pose = m_drive.getPose();
        telemetry.addData("Drive Mode", m_fieldCentric ? "Field" : "Robot");
        telemetry.addData("tracking power", m_turret.computeTrackingPower());
        telemetry.addData("Pose X", pose.getX());
        telemetry.addData("Pose Y", pose.getY());
        telemetry.addData("Heading Deg", pose.getHeading());
        telemetry.addData("Hood position: ", m_hood.getCurrentPwm());
        telemetry.addData("tag pos", m_limelight.getTX()+ " " + m_limelight.getTY());
        telemetry.update();
    }
}
