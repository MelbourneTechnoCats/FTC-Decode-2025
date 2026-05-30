package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Vision subsystem backed directly by Limelight 3A's built‑in AprilTag pipeline.
 *
 * - No VisionPortal/Webcam usage.
 * - Estimates robot pose from multiple field tags.
 * - Tracks "Obelisk" tags for motif detection.
 * - Exposes blue/red target poses & ranges.
 *
 * NOTE: All positions here are in the field coordinate system and units
 * as reported by Limelight (meters). If you need inches, convert in callers.
 */
public class VisionSubsystem extends SubsystemBase {

    private final Telemetry m_telemetry;
    private final Limelight3A m_limelight;

    // Last estimated robot pose (field frame, meters & radians)
    private double m_xPosition = 0.0;
    private double m_yPosition = 0.0;
    private double m_headingRad = 0.0;

    // Target tag poses (from LLResultTypes.FiducialResult)
    // We keep them in meters from the field origin.
    private AprilTagPoseFtcLite m_redTargetPose;
    private AprilTagPoseFtcLite m_blueTargetPose;

    /** Minimal AprilTag pose holder similar to AprilTagPoseFtc but without VisionPortal. */
    public static class AprilTagPoseFtcLite {
        public final double x;      // meters, field coords
        public final double y;      // meters, field coords
        public final double z;      // meters (height), if you care
        public final double range;  // meters, distance robot->tag (approx)
        public final double bearingDeg; // bearing from robot to tag, deg

        public AprilTagPoseFtcLite(double x, double y, double z,
                                   double range, double bearingDeg) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.range = range;
            this.bearingDeg = bearingDeg;
        }
    }

    public class PoseTrigger extends Trigger {
        boolean m_update = false;

        @Override
        public boolean get() {
            return m_update;
        }
    }
    public final PoseTrigger m_poseTrigger = new PoseTrigger();

    public enum Motif {
        NONE,
        GPP,
        PGP,
        PPG
    }

    private Motif m_motif = Motif.NONE;

    public VisionSubsystem(final HardwareMap hardwareMap, final Telemetry telemetry) {
        m_telemetry = telemetry;
        m_limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Configure Limelight here if desired (pipeline, LEDs, etc).
        // Example:
        // m_limelight.pipelineSwitch(0);
        // m_limelight.setLEDMode(Limelight3A.LedMode.ON);
    }

    @Override
    public void periodic() {
        m_blueTargetPose = null;
        m_redTargetPose = null;
        m_poseTrigger.m_update = false;

        LLResult result = m_limelight.getLatestResult();
        if (result == null) {
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return;

        double meanX = 0, meanY = 0;
        double meanEndX = 0, meanEndY = 0;
        int numPoints = 0;

        for (LLResultTypes.FiducialResult f : fiducials) {
            int id = f.getFiducialId();

            // Field tag pose in meters (Limelight coordinate frame).
            double tagX = f.getTargetPoseRobotSpace().getPosition().x; // robot-space, but we can still use to approximate heading
            double tagY = f.getTargetPoseRobotSpace().getPosition().y;

            // For robot pose estimation we still use the original averaging trick,
            // but now based on robot-space vectors.
            // Compute an approximate robot pose in a 2D plane:
            // heading is direction from robot->tag; we build a unit vector from that.
            double dx = tagX;
            double dy = tagY;
            double headingRad = Math.atan2(dy, dx);

            // approximate "field" position of robot relative to tag
            // by inverting the vector:
            double robotX = -dx;
            double robotY = -dy;

            meanX += robotX;
            meanY += robotY;

            Vector2d vec = new Vector2d(1.0, 0.0).rotateBy(Math.toDegrees(headingRad));
            meanEndX += robotX + vec.getX();
            meanEndY += robotY + vec.getY();
            numPoints++;

            // Identify blue/red obelisk tags by ID
            if (id == 20) { // blue obelisk
                m_blueTargetPose = makePoseFromFiducial(f);
            } else if (id == 24) { // red obelisk
                m_redTargetPose = makePoseFromFiducial(f);
            }

            // Motif detection from obelisk tag IDs 21–23
            if (m_motif == Motif.NONE) {
                switch (id) {
                    case 21: m_motif = Motif.GPP; break;
                    case 22: m_motif = Motif.PGP; break;
                    case 23: m_motif = Motif.PPG; break;
                    default: break;
                }
            }
        }

        if (numPoints > 0) {
            meanX /= numPoints;
            meanY /= numPoints;
            meanEndX /= numPoints;
            meanEndY /= numPoints;

            m_xPosition = meanX;
            m_yPosition = meanY;

            Vector2d vec = new Vector2d(meanEndX - meanX, meanEndY - meanY);
            m_headingRad = vec.angle(); // already radians

            m_poseTrigger.m_update = true;
        }
    }

    /** Convert Limelight fiducial data into our lightweight pose. */
    private AprilTagPoseFtcLite makePoseFromFiducial(LLResultTypes.FiducialResult f) {
        // Robot-space coordinates (m)
        double rx = f.getTargetPoseRobotSpace().getPosition().x;
        double ry = f.getTargetPoseRobotSpace().getPosition().y;
        double rz = f.getTargetPoseRobotSpace().getPosition().z;

        double range = Math.hypot(rx, ry);
        double bearingDeg = Math.toDegrees(Math.atan2(ry, rx));

        // For now we store pose in robot frame; callers that compare with robot pose
        // should use range/bearing rather than x/y directly.
        return new AprilTagPoseFtcLite(rx, ry, rz, range, bearingDeg);
    }

    /** Last estimated robot pose in *robot frame units* (meters, radians). */
    public Pose2d getLastPose() {
        return new Pose2d(m_xPosition, m_yPosition, new Rotation2d(m_headingRad));
    }

    public AprilTagPoseFtcLite getRedTargetPose() {
        return m_redTargetPose;
    }

    public AprilTagPoseFtcLite getBlueTargetPose() {
        return m_blueTargetPose;
    }

    public Limelight3A getLimelight() {
        return m_limelight;
    }

    /** Range to red target in meters. */
    public double getRedTargetRange() {
        return (m_redTargetPose != null) ? m_redTargetPose.range : Double.NaN;
    }

    /** Range to blue target in meters. */
    public double getBlueTargetRange() {
        return (m_blueTargetPose != null) ? m_blueTargetPose.range : Double.NaN;
    }

    public Motif getMotif() {
        return m_motif;
    }
}