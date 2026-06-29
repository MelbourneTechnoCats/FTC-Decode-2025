import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

/**
 * Simple manual turret test WITHOUT using a default command:
 * - Right stick X (gamepad2) controls turret power
 * - Telemetry shows turret velocity
 */
@TeleOp(name = "Test: Turret Manual", group = "Test")
@Config
public class TurretOpMode extends CommandOpMode {

    private GamepadEx m_operator;
    private TurretSubsystem m_turret;

    // Scale for manual control
    public static double turretScale = 0.5;

    public static double squareInput(double input) {
        return Math.copySign(input * input, input);
    }

    @Override
    public void initialize() {
        m_operator = new GamepadEx(gamepad2);

        // Manual-only turret (no vision)
        m_turret = new TurretSubsystem(hardwareMap, telemetry);

        // Register turret so its periodic() runs
        register(m_turret);
    }

    @Override
    public void run() {
        // Let CommandOpMode do its normal scheduling work
        super.run();

        if (!isStarted() || isStopRequested()) {
            return;
        }

        // Manual control each loop – NO default command
        double rx = m_operator.getRightX();
        double power = squareInput(rx) * turretScale;

        m_turret.setPower(power*2000);

        telemetry.addData("Turret Power", power);
        telemetry.update();
    }
}
