package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Minimal Limelight 3A wrapper.
 *
 * In this project we don't have a real Limelight, so this class doubles as a
 * vision helper that can derive heading to an AprilTag using the existing
 * VisionSubsystem (FTC AprilTag pipeline).
 */
public class LimelightSubsystem extends SubsystemBase {
    private final Telemetry m_telemetry;
    private final VisionSubsystem m_vision;
    private final DriveSubsystem m_drive;

    // Last known values (Limelight-style)
    private double m_tx = 0.0;   // horizontal offset (deg)
    private double m_ty = 0.0;   // vertical offset (deg)
    private boolean m_targetVisible = false;

    public LimelightSubsystem(Telemetry telemetry, VisionSubsystem vision, DriveSubsystem drive) {
        m_telemetry = telemetry;
        m_vision = vision;
        m_drive = drive;
    }

    @Override
    public void periodic() {
        // In a real Limelight setup, you'd pull tx/ty/targetValid here.
        // For now we just publish whatever values were last set.
        m_telemetry.addData("Limelight tx", m_tx);
        m_telemetry.addData("Limelight ty", m_ty);
        m_telemetry.addData("Limelight target", m_targetVisible);
    }

    public double getTx() {
        return m_tx;
    }

    public double getTy() {
        return m_ty;
    }

     public boolean hasTarget() {
        return m_targetVisible;
    }

    /**
     * Actively looks for an AprilTag using the existing VisionSubsystem and returns
     * the robot heading (in degrees) toward that tag.
     *
     * @param blue if true, look for the blue obelisk tag; otherwise look for red
     * @return heading in degrees from robot to tag in the FTC field frame,
     *         or Double.NaN if no suitable tag is visible
     */
     public double getHeadingToAprilTag(boolean blue) {
        // Get robot pose in field frame
        com.arcrobotics.ftclib.geometry.Pose2d robotPose = m_drive.getPose();

        // Get target pose in field frame from VisionSubsystem
        org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc tagPose =
                blue ? m_vision.getBlueTargetPose() : m_vision.getRedTargetPose();

        if (tagPose == null) {
            m_targetVisible = false;
            m_tx = 0.0;
            m_ty = 0.0;
            return Double.NaN;
        }

        // Tag pose is in field coordinates in inches (FTC). Convert to same units as robotPose (inches vs meters).
        // VisionSubsystem stores last pose also in inches, so we stay consistent.
        double tagX = tagPose.x;
        double tagY = tagPose.y;

        double dx = tagX - robotPose.getX();
        double dy = tagY - robotPose.getY();

        double headingToTag = Math.toDegrees(Math.atan2(dy, dx));

        m_targetVisible = true;
        m_tx = 0.0;
        m_ty = 0.0;

        return headingToTag;
    }

    /**
     * Compute a compensated heading to the AprilTag that leads the shot based on robot motion.
     *
     * @param blue          true for blue obelisk tag, false for red
     * @param projectileVel projectile speed (same units/sec as field coordinates; tune from shooter)
     * @param leadScale     additional tuning multiplier on lead amount (1.0 = nominal)
     * @return desired field heading in degrees to aim turret, or NaN if no tag
     */
    public double getLeadHeadingToAprilTag(boolean blue, double projectileVel, double leadScale) {
        // Get basic heading and distance to tag
        double range = blue ? m_vision.getBlueTargetRange() : m_vision.getRedTargetRange();
        if (Double.isNaN(range) || projectileVel <= 0) {
            return getHeadingToAprilTag(blue);
        }

        com.arcrobotics.ftclib.geometry.Pose2d robotPose = m_drive.getPose();
        org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc tagPose =
                blue ? m_vision.getBlueTargetPose() : m_vision.getRedTargetPose();
        if (tagPose == null) {
            return Double.NaN;
        }

        double tagX = tagPose.x;
        double tagY = tagPose.y;

        // Vector robot -> tag
        double rx = tagX - robotPose.getX();
        double ry = tagY - robotPose.getY();
        double dist = Math.hypot(rx, ry);
        if (dist < 1e-6) {
            return Math.toDegrees(robotPose.getHeading());
        }

        // Robot field velocity
        com.arcrobotics.ftclib.geometry.Vector2d vField = m_drive.getFieldVelocity();

        // Estimate time of flight and lead vector
        double tFlight = dist / projectileVel;
        tFlight *= leadScale;

        double leadX = rx - vField.getX() * tFlight;
        double leadY = ry - vField.getY() * tFlight;

        double leadHeading = Math.toDegrees(Math.atan2(leadY, leadX));
        return leadHeading;
    }

    // Optional setters so you can simulate from Dashboard or test code
    public void setFakeReading(double tx, double ty, boolean visible) {
        m_tx = tx;
        m_ty = ty;
        m_targetVisible = visible;
    }
}