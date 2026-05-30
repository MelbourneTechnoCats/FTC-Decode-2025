//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.command.Command;
//import com.arcrobotics.ftclib.command.FunctionalCommand;
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
///**
// * Turret subsystem using a single DC motor.
// *
// * - Manual control: open-loop power [-1, 1].
// * - Auto-aim: simple P loop from heading error (deg) -> motor power.
// */
//@Config
//public class TurretSubsystem extends SubsystemBase {
//    private final MotorSubsystem m_turretMotor;
//    private final LimelightSubsystem m_limelight;
//    private final Telemetry m_telemetry;
//
//    // Motor/encoder configuration
//    public static double kTurretEncoderResolution = 28.0; // ticks per motor rev (before gearing)
//    public static boolean kTurretInverted = false;
//
//        // Auto-aim tuning
//    public static double kAimP = 0.02;       // power per degree of error
//    public static double kMaxAutoPower = 0.4;
//
//    // Scan behavior
//    public static double kScanPower = 0.15;      // turret scan power (tune)
//    public static double kLostTargetTimeoutMs = 300; // how long w/o target before we go back to scan
//
//    // Projectile and lead tuning
//    public static double kProjectileSpeed = 5.0; // field units / s
//    public static double kLeadScale = 1.0;
//
//    // Which AprilTag to track by default
//    public static boolean kTrackBlueTag = true;
//
//    public TurretSubsystem(final HardwareMap hardwareMap,
//                           final Telemetry telemetry,
//                           final LimelightSubsystem limelight) {
//        m_telemetry = telemetry;
//        m_limelight = limelight;
//
//        // turretMotor should exist in the Robot Configuration
//        m_turretMotor = new MotorSubsystem(
//                hardwareMap,
//                "turretMotor",
//                kTurretEncoderResolution,
//                kTurretInverted
//        );
//    }
//
//    @Override
//    public void periodic() {
//        m_telemetry.addLine("Turret:")
//                .addData("velRPM", m_turretMotor.getVelocity())
//                .addData("LL hasTarget", m_limelight != null && m_limelight.hasTarget());
//    }
//
//    /**
//     * Manual turret control with open-loop power from -1..1.
//     * The caller provides a DoubleSupplier (e.g. gamepad stick x).
//     */
//    public Command manualCommand(java.util.function.DoubleSupplier powerSupplier) {
//        return m_turretMotor.setPowerCommand(powerSupplier::getAsDouble);
//    }
//
//    /**
//     * Stop the turret motor.
//     */
//    public Command stopCommand() {
//        return m_turretMotor.setPowerCommand(0.0);
//    }
//     /**
//     * Simple scanning command: rotate turret at a fixed power to look for a tag.
//     */
//    public Command scanCommand() {
//        return m_turretMotor.setPowerCommand(() -> kScanPower);
//    }
//    public Command trackTargetCommand() {
//        return new FunctionalCommand(
//                () -> { /* init: nothing */ },
//                () -> {
//                    if (m_limelight == null || m_limelight.getDriveSubsystem() == null) {
//                        m_turretMotor.setPowerCommand(0.0).schedule();
//                        return;
//                    }
//
//                    double leadHeading = m_limelight.getLeadHeadingToAprilTag(
//                            kTrackBlueTag, kProjectileSpeed, kLeadScale
//                    );
//                    if (Double.isNaN(leadHeading)) {
//                        // No valid target; hold still
//                        m_turretMotor.setPowerCommand(0.0).schedule();
//                        return;
//                    }
//
//                    // Robot heading from drive (field frame, radians)
//                    com.arcrobotics.ftclib.geometry.Pose2d robotPose =
//                            m_limelight.getDriveSubsystem().getPose();
//                    double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());
//
//                    // Error is how far off robot heading is from desired aiming heading
//                    double errorDeg = leadHeading - robotHeadingDeg;
//
//                    // Normalize to [-180, 180]
//                    errorDeg = ((errorDeg + 180) % 360 + 360) % 360 - 180;
//
//                    double power = kAimP * errorDeg;
//                    if (power > kMaxAutoPower) power = kMaxAutoPower;
//                    if (power < -kMaxAutoPower) power = -kMaxAutoPower;
//
//                    m_turretMotor.setPowerCommand(power).schedule();
//                },
//                interrupted -> m_turretMotor.setPowerCommand(0.0).schedule(),
//                () -> false,
//                this
//        );
//    }
//
//    /**
//     * Full behavior:
//     * - If no AprilTag: slowly scan 360° (open-loop power).
//     * - When a tag is seen: switch to closed-loop tracking.
//     * - If tag is lost for some time: go back to scanning.
//     *
//     * Run this as your main turret command in TeleOp/Auto.
//     */
//    public Command autoTrackWithScanCommand() {
//        return new FunctionalCommand(
//                // init
//                () -> {
//                    m_turretMotor.setPowerCommand(0.0).schedule();
//                    m_lastSeenTime = 0;
//                    m_tracking = false;
//                },
//                // execute
//                () -> {
//                    if (m_limelight == null) {
//                        m_turretMotor.setPowerCommand(0.0).schedule();
//                        return;
//                    }
//
//                    boolean hasTarget = m_limelight.hasTarget();
//                    long now = System.currentTimeMillis();
//
//                    if (hasTarget) {
//                        // record last time we saw tag
//                        m_lastSeenTime = now;
//                    }
//
//                    if (!m_tracking) {
//                        // Currently scanning
//                        if (hasTarget) {
//                            // Switch into tracking mode
//                            m_tracking = true;
//                        } else {
//                            // keep scanning
//                            m_turretMotor.setPowerCommand(kScanPower).schedule();
//                            return;
//                        }
//                    } else {
//                        // Currently tracking
//                        if (!hasTarget && m_lastSeenTime > 0 &&
//                                now - m_lastSeenTime > kLostTargetTimeoutMs) {
//                            // lost tag -> go back to scanning
//                            m_tracking = false;
//                            m_turretMotor.setPowerCommand(kScanPower).schedule();
//                            return;
//                        }
//                    }
//
//                    // If we reach here and m_tracking == true, run the aim loop inline.
//                    // (Reuse the same logic as trackTargetCommand)
//                    double leadHeading = m_limelight.getLeadHeadingToAprilTag(
//                            kTrackBlueTag, kProjectileSpeed, kLeadScale
//                    );
//                    if (Double.isNaN(leadHeading)) {
//                        m_turretMotor.setPowerCommand(0.0).schedule();
//                        return;
//                    }
//
//                    com.arcrobotics.ftclib.geometry.Pose2d robotPose =
//                            m_limelight.getDriveSubsystem().getPose();
//                    double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());
//
//                    double errorDeg = leadHeading - robotHeadingDeg;
//                    errorDeg = ((errorDeg + 180) % 360 + 360) % 360 - 180;
//
//                    double power = kAimP * errorDeg;
//                    if (power > kMaxAutoPower) power = kMaxAutoPower;
//                    if (power < -kMaxAutoPower) power = -kMaxAutoPower;
//
//                    m_turretMotor.setPowerCommand(power).schedule();
//                },
//                // end
//                interrupted -> m_turretMotor.setPowerCommand(0.0).schedule(),
//                // never finishes on its own
//                () -> false,
//                this
//        );
//    }
//
//    // --- internal state for autoTrackWithScanCommand ---
//    private long m_lastSeenTime = 0;
//    private boolean m_tracking = false;
//}
