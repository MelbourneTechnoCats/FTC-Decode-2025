package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Basic turret subsystem:
 *  - one motor for rotation
 *  - Limelight 3A camera (via LimelightSubsystem) for heading to AprilTags
 *
 * Uses open-loop power for manual control and a simple P loop on heading error
 * (robot heading vs heading to target AprilTag) for auto-aim.
 */
@Config
public class TurretSubsystem extends SubsystemBase {
    private final MotorSubsystem m_turretMotor;
    private final LimelightSubsystem m_limelight;
    private final Telemetry m_telemetry;

    // Encoder resolution for turret motor (ticks per rev) – update to your motor/gearbox
    public static double kTurretEncoderResolution = 28.0;

    // Simple P gain converting heading error (deg) into motor power
    public static double kAimP = 0.02;

    // Estimated projectile (artifact) speed in field units per second (tune experimentally)
    public static double kProjectileSpeed = 5.0;

    // Extra scaling for lead angle compensation
    public static double kLeadScale = 1.0;

    // Max allowed turret power during auto-aim
    public static double kMaxAutoPower = 0.4;

    // Which AprilTag to track by default
    public static boolean kTrackBlueTag = true;

    public TurretSubsystem(final HardwareMap hardwareMap,
                           final Telemetry telemetry,
                           final LimelightSubsystem limelight) {
        m_telemetry = telemetry;
        m_limelight = limelight;

        // turretMotor should exist in the Robot Configuration
        m_turretMotor = new MotorSubsystem(
                hardwareMap,
                "turretMotor",
                kTurretEncoderResolution
        );
    }

    @Override
    public void periodic() {
        // Telemetry for debugging
        m_telemetry.addLine("Turret:")
                .addData("vel", m_turretMotor.getVelocity())
                .addData("LL hasTarget", m_limelight.hasTarget());
    }

    /**
     * Manual turret control with open-loop power from -1..1.
     */
    public Command manualCommand(java.util.function.DoubleSupplier powerSupplier) {
        return m_turretMotor.setPowerCommand(powerSupplier::getAsDouble);
    }

    /**
     * Stop the turret motor.
     */
    public Command stopCommand() {
        return m_turretMotor.setPowerCommand(0.0);
    }

    /**
     * Track an AprilTag while this command is scheduled.
     * Uses LimelightSubsystem.getHeadingToAprilTag(...) to get the heading (deg)
     * toward the selected tag and drives the turret to minimize that heading.
     *
     * Positive heading means robot is rotated one way relative to tag; we interpret
     * that as an error and drive the turret proportionally.
     */
   public Command trackTargetCommand() {
        return new FunctionalCommand(
                () -> { /* init: nothing */ },
                () -> {
                    double leadHeading = m_limelight.getLeadHeadingToAprilTag(kTrackBlueTag, kProjectileSpeed, kLeadScale);
                    if (Double.isNaN(leadHeading)) {
                        m_turretMotor.setPowerCommand(0.0).schedule();
                        return;
                    }

                    // Robot heading from drive/vision (field frame)
                    // We use LimelightSubsystem's drive reference
                    com.arcrobotics.ftclib.geometry.Pose2d robotPose =
                            m_limelight != null ? m_limelight.m_vision.getLastPose() : null;
                    if (robotPose == null) {
                        m_turretMotor.setPowerCommand(0.0).schedule();
                        return;
                    }
                    double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());

                    // Desired turret angle relative to robot = leadHeading - robotHeading
                    double errorDeg = leadHeading - robotHeadingDeg;

                    // Normalize to [-180, 180] to avoid long rotation
                    errorDeg = ((errorDeg + 180) % 360 + 360) % 360 - 180;

                    double power = kAimP * errorDeg;
                    if (power > kMaxAutoPower) power = kMaxAutoPower;
                    if (power < -kMaxAutoPower) power = -kMaxAutoPower;

                    m_turretMotor.setPowerCommand(power).schedule();
                },
                interrupted -> {
                    m_turretMotor.setPowerCommand(0.0).schedule();
                },
                () -> false,
                this
        );
    }
}