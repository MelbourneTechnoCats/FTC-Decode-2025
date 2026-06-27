package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Turret subsystem using simple & fast Limelight tx-based tracking (Method 1).
 * This is the recommended starting approach — very responsive and stable.
 */
@Config
public class TurretSubsystem extends SubsystemBase {

    private final MotorSubsystem m_turretMotor;
    private final VisionSubsystem m_vision;
    private final Telemetry m_telemetry;

    // ==================== Tunables ====================
    public static double kTurretEncoderResolution = 28.0;
    public static boolean kTurretInverted = false;

    public static double kAimP = 0.028;           // Power per degree of tx error
    public static double kFeedForward = 0.08;     // Small constant to overcome friction
    public static double kMaxAutoPower = 0.50;

    public static double kScanPower = 0.18;
    public static double kLostTargetTimeoutMs = 400;

    public static boolean kTrackBlueTag = true;

    // Deadband to prevent jitter
    public static double kDeadbandDeg = 2.0;

    private long m_lastSeenTime = 0;
    private boolean m_isTracking = false;

    public TurretSubsystem(HardwareMap hardwareMap,
                           Telemetry telemetry,
                           VisionSubsystem vision) {
        m_telemetry = telemetry;
        m_vision = vision;

        m_turretMotor = new MotorSubsystem(
                hardwareMap,
                "turretMotor",
                kTurretEncoderResolution,
                kTurretInverted
        );
    }

    /**
     * Constructor for manual-only control (no vision tracking).
     */
    public TurretSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, null);
    }

    @Override
    public void periodic() {
        m_telemetry.addLine("Turret:")
                .addData("velRPM", m_turretMotor.getVelocity())
                .addData("Tracking", m_isTracking)
                .addData("tx", getCurrentTx());
    }

    /**
     * Directly set the turret motor power.
     */
    public void setPower(double power) {
        m_turretMotor.setRawPower(power);
    }

    private double getCurrentTx() {
        return m_vision != null ? m_vision.getTargetTx(kTrackBlueTag) : Double.NaN;
    }

    // ====================== Commands ======================

    public Command manualCommand(java.util.function.DoubleSupplier powerSupplier) {
        return m_turretMotor.setPowerCommand(powerSupplier::getAsDouble);
    }

    public Command stopCommand() {
        return m_turretMotor.setPowerCommand(0.0);
    }

    public Command scanCommand() {
        return m_turretMotor.setPowerCommand(() -> kScanPower);
    }

    /**
     * Core tracking logic using Limelight tx.
     */
    private double computeTrackingPower() {
        if (m_vision == null) return Double.NaN;

        double tx = getCurrentTx();
        if (Double.isNaN(tx)) return Double.NaN;

        // Simple P + Feedforward
        double power = kAimP * tx;

        // Add small feedforward to overcome static friction
        if (Math.abs(tx) > kDeadbandDeg) {
            power += Math.signum(tx) * kFeedForward;
        }

        // Clamp
        if (power > kMaxAutoPower) power = kMaxAutoPower;
        if (power < -kMaxAutoPower) power = -kMaxAutoPower;

        return power;
    }

    /**
     * Pure closed-loop tracking command (tracks when target visible).
     */
    public Command trackTargetCommand() {
        return new FunctionalCommand(
                () -> m_turretMotor.setRawPower(0.0),
                () -> {
                    double power = computeTrackingPower();
                    m_turretMotor.setRawPower(Double.isNaN(power) ? 0.0 : power);
                },
                interrupted -> m_turretMotor.setRawPower(0.0),
                () -> false,
                this
        );
    }

    /**
     * Best command: Auto scan when no target, auto-track when target acquired.
     */
    public Command autoTrackWithScanCommand() {
        return new FunctionalCommand(
                () -> {
                    m_turretMotor.setRawPower(0.0);
                    m_lastSeenTime = 0;
                    m_isTracking = false;
                },
                () -> {
                    double tx = getCurrentTx();
                    boolean hasTarget = !Double.isNaN(tx);
                    long now = System.currentTimeMillis();

                    //     get target
                    //     start tracking






                    //clean up
                    // if (!m_isTracking) {
                    //     if (hasTarget) {
                    //         m_isTracking = true; // once this is set the remaining if block is skipped
                    //     } else {
                    //         m_turretMotor.setRawPower(kScanPower);
                    //         return;
                    //     }
                    // } else {
                    //     if (!hasTarget && m_lastSeenTime > 0 &&
                    //             now - m_lastSeenTime > kLostTargetTimeoutMs) {
                    //         m_isTracking = false;
                    //         m_turretMotor.setRawPower(kScanPower);
                    //         return;
                    //     }
                    // }

                    double power = computeTrackingPower();
                    m_turretMotor.setRawPower(Double.isNaN(power) ? 0.0 : power);
                },
                interrupted -> m_turretMotor.setRawPower(0.0),
                () -> false,
                this
        );
    }

    public void setTrackBlue(boolean trackBlue) {
        kTrackBlueTag = trackBlue;
    }
}