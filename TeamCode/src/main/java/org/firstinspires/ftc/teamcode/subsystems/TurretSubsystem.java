package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Turret subsystem using simple & fast Limelight tx-based tracking (Method 1).
 * This is the recommended starting approach — very responsive and stable.
 */
@Config
public class TurretSubsystem extends SubsystemBase {

    private final MotorSubsystem m_turretMotor;
    private final LimelightSubsystem m_vision;
    private final Telemetry m_telemetry;

    // ==================== Tunables ====================
    public static double kTurretEncoderResolution = 28.0;
    public static boolean kTurretInverted = false;

    public static double kAimP = 0.1;           // Power per degree of tx error
    public static double kFeedForward = 0.08;     // Small constant to overcome friction
    public static double kMaxAutoPower = 1;

    public static double kScanPower = 0.18;
    public static double kLostTargetTimeoutMs = 400;

    public static boolean kTrackBlueTag = true;

    // Deadband to prevent jitter
    public static double kDeadbandDeg = 2.0;

    private long m_lastSeenTime = 0;
    private boolean m_isTracking = false;

    public TurretSubsystem(HardwareMap hardwareMap,
                           Telemetry telemetry,
                           LimelightSubsystem vision) {
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
                .addData("tx", getCurrentTx()).addData("power", m_turretMotor.getPower());
    }

    /**
     * Directly set the turret motor power.
     */
    public void setPower(double power) {
        m_turretMotor.setRawPocwer(power);
    }

    private double getCurrentTx() {
        return m_vision != null ? m_vision.getTX() : Double.NaN;
    }

    // ====================== Commands ======================

    public Command manualCommand(java.util.function.DoubleSupplier powerSupplier) {
        return m_turretMotor.setPowerCommand(powerSupplier);
    }

    public Command stopCommand() {
        return m_turretMotor.setPowerCommand(0.0);
    }

    // public Command scanCommand() {
    //     return m_turretMotor.setPowerCommand(() -> kScanPower);
    // }

    /**
     * Core tracking logic using Limelight tx.
     */
    public double computeTrackingPower() {
        if (m_vision == null) return 0;

        double tx = getCurrentTx();
        if (Double.isNaN(tx)) return 0;
        if (!m_vision.hasTarget()) return 0;


        double power = kAimP * tx;

        // Add small feedforward to overcome static friction
        if (Math.abs(tx) < kDeadbandDeg) {
            power += Math.signum(tx) * kFeedForward;
        }

        // Clamp
        if (power > kMaxAutoPower) power = kMaxAutoPower;
        if (power < -kMaxAutoPower) power = -kMaxAutoPower;

        return power;
    }



    public Command trackTarget(){

        /**
         * if no tag: rotate to find a tag at a slow speed
         * if find tag: stop scan, try to minimize tX with PID
         *
         * */
       return new RunCommand(
               () -> {
                   m_turretMotor.setRawPower(computeTrackingPower());
               }
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
