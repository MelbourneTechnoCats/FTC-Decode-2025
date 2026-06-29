package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

/**
 * Vision subsystem backed by a Limelight 3A.
 *
 * <p>Responsibilities:
 * <ul>
 *   <li>estimate robot pose using Limelight botpose / MegaTag2,</li>
 *   <li>track the red and blue AprilTag targets,</li>
 *   <li>detect the current motif from specific field tags, and</li>
 *   <li>expose a trigger when a fresh pose estimate is available.</li>
 * </ul>
 *
 * <p>Prerequisites:
 * <ol>
 *   <li>Configure robot pose (LL Forward, Right, Up, Roll/Pitch/Yaw) in the Limelight web UI.</li>
 *   <li>Upload the correct field map (Into The Deep) via the Limelight web UI.</li>
 *   <li>Use pipeline 0, or the AprilTag pipeline configured for this camera.</li>
 * </ol>
 */
public class VisionSubsystem extends SubsystemBase {

    private final Telemetry m_telemetry;
    private final Limelight3A m_limelight;
    private final IMU m_imu; // Required for MegaTag2
    private final GoBildaPinpointDriver m_pinpoint;

    // Last estimated robot pose (field frame, meters & radians)
    private Pose2d m_lastPose = new Pose2d();

    // Target tag poses
    private AprilTagPoseFtcLite m_redTargetPose;
    private AprilTagPoseFtcLite m_blueTargetPose;

    /**
     * Lightweight AprilTag pose data for target tracking.
     */
    public static class AprilTagPoseFtcLite {
        /** X position in meters. */
        public final double x;
        /** Y position in meters. */
        public final double y;
        /** Z position in meters. */
        public final double z;
        /** Planar range to the target in meters. */
        public final double range;
        /** Bearing to the target in degrees. */
        public final double bearingDeg;

        public AprilTagPoseFtcLite(double x, double y, double z, double range, double bearingDeg) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.range = range;
            this.bearingDeg = bearingDeg;
        }
    }

    public class PoseTrigger extends Trigger {
        private boolean m_update = false;

        @Override
        public boolean get() {
            return m_update;
        }

        /**
         * Updates whether a new pose estimate is available.
         *
         * @param update true when a new pose was received
         */
        public void setUpdate(boolean update) {
            m_update = update;
        }
    }

    public final PoseTrigger m_poseTrigger = new PoseTrigger();

    public enum Motif {
        NONE, GPP, PGP, PPG
    }

    private Motif m_motif = Motif.NONE;

    /**
     * Creates the vision subsystem using a standard IMU.
     *
     * @param hardwareMap hardware map used to retrieve the Limelight
     * @param telemetry telemetry for optional diagnostics
     * @param imu robot IMU used for MegaTag2 orientation updates; may be null
     */
    public VisionSubsystem(final HardwareMap hardwareMap, final Telemetry telemetry, IMU imu) {
        m_telemetry = telemetry;
        m_limelight = hardwareMap.get(Limelight3A.class, "limelight");
        m_imu = imu;
        m_pinpoint = null;

        // Recommended initial setup
        m_limelight.pipelineSwitch(0); // AprilTag pipeline
        m_limelight.start();           // Start polling data
    }

    /**
     * Creates the vision subsystem using a goBILDA Pinpoint computer as the IMU source.
     *
     * @param hardwareMap hardware map
     * @param telemetry telemetry
     * @param pinpoint Pinpoint driver instance
     */
    public VisionSubsystem(final HardwareMap hardwareMap, final Telemetry telemetry, GoBildaPinpointDriver pinpoint) {
        m_telemetry = telemetry;
        m_limelight = hardwareMap.get(Limelight3A.class, "limelight");
        m_imu = null;
        m_pinpoint = pinpoint;

        m_limelight.pipelineSwitch(0);
        m_limelight.start();
    }

    /**
     * Convenience constructor that fetches the Pinpoint from the hardware map.
     */
    public VisionSubsystem(final HardwareMap hardwareMap, final Telemetry telemetry) {
        this(hardwareMap, telemetry, hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint"));
    }

    @Override
    public void periodic() {
        m_blueTargetPose = null;
        m_redTargetPose = null;
        m_poseTrigger.setUpdate(false);
        m_motif = Motif.NONE; // Reset every cycle or keep last known?

        LLResult result = m_limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        // Robot pose estimation in the field frame.
        // Prefer MegaTag2 when the IMU/Pinpoint is available.
        Pose3D botPose;
        if (m_imu != null) {
            YawPitchRollAngles orientation = m_imu.getRobotYawPitchRollAngles();
            m_limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
            botPose = result.getBotpose_MT2();
        } else if (m_pinpoint != null) {
            m_pinpoint.update();
            m_limelight.updateRobotOrientation(m_pinpoint.getHeading(AngleUnit.DEGREES));
            botPose = result.getBotpose_MT2();
        } else {
            botPose = result.getBotpose();
        }

        if (botPose != null) {
            double x = botPose.getPosition().x;
            double y = botPose.getPosition().y;
            double yawRad = Math.toRadians(botPose.getOrientation().getYaw());

            m_lastPose = new Pose2d(x, y, new Rotation2d(yawRad));
            m_poseTrigger.setUpdate(true);
        }

        // Process individual fiducials for targets and motif detection.
        if (fiducials != null) {
            for (LLResultTypes.FiducialResult f : fiducials) {
                int id = f.getFiducialId();

                // Blue / Red target tags.
                if (id == 20) { // blue
                    m_blueTargetPose = makePoseFromFiducial(f);
                } else if (id == 24) { // red
                    m_redTargetPose = makePoseFromFiducial(f);
                }

                // Motif detection tags.
                if (m_motif == Motif.NONE) {
                    switch (id) {
                        case 21: m_motif = Motif.GPP; break;
                        case 22: m_motif = Motif.PGP; break;
                        case 23: m_motif = Motif.PPG; break;
                    }
                }
            }
        }
    }

    /**
     * Converts a fiducial detection into a lightweight robot-space pose.
     *
     * @param f detected fiducial result
     * @return pose data with range and bearing
     */
    private AprilTagPoseFtcLite makePoseFromFiducial(LLResultTypes.FiducialResult f) {
        Position pos = f.getTargetPoseRobotSpace().getPosition();
        double rx = pos.x;
        double ry = pos.y;
        double rz = pos.z;
        double range = Math.hypot(rx, ry);
        double bearingDeg = Math.toDegrees(Math.atan2(ry, rx));

        return new AprilTagPoseFtcLite(rx, ry, rz, range, bearingDeg);
    }

    /** @return the last estimated robot pose */
    public Pose2d getLastPose() {
        return m_lastPose;
    }

    /** @return the most recent red target pose, or null if not visible */
    public AprilTagPoseFtcLite getRedTargetPose() {
        return m_redTargetPose;
    }

    /** @return the most recent blue target pose, or null if not visible */
    public AprilTagPoseFtcLite getBlueTargetPose() {
        return m_blueTargetPose;
    }

    /** @return red target range in meters, or NaN if the target is not visible */
    public double getRedTargetRange() {
        return m_redTargetPose != null ? m_redTargetPose.range : Double.NaN;
    }

    /** @return blue target range in meters, or NaN if the target is not visible */
    public double getBlueTargetRange() {
        return m_blueTargetPose != null ? m_blueTargetPose.range : Double.NaN;
    }

    /** @return the current motif, or NONE when no matching tag is visible */
    public Motif getMotif() {
        return m_motif;
    }

    /** @return the Limelight camera instance used by this subsystem */
    public Limelight3A getLimelight() {
        return m_limelight;
    }

    /**
     * Manually pushes IMU/Pinpoint yaw to Limelight for MegaTag2 processing.
     */
    public void updateRobotOrientation() {
        if (m_imu != null) {
            YawPitchRollAngles angles = m_imu.getRobotYawPitchRollAngles();
            m_limelight.updateRobotOrientation(angles.getYaw(AngleUnit.DEGREES));
        } else if (m_pinpoint != null) {
            m_pinpoint.update();
            m_limelight.updateRobotOrientation(m_pinpoint.getHeading(AngleUnit.DEGREES));
        }
    }
    // Add at the bottom with other getters
/**
 * Returns the horizontal offset (tx) in degrees for the selected target.
 * Positive = tag is to the right of camera center.
 * Negative = tag is to the left.
 */ 
public double getTargetTx(boolean trackBlue) {
    AprilTagPoseFtcLite target = trackBlue ? getBlueTargetPose() : getRedTargetPose();
    if (target == null) return Double.NaN;

    LLResult result = m_limelight.getLatestResult();
    if (result == null || !result.isValid()) return Double.NaN;

    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
    if (fiducials == null) return Double.NaN;

    int desiredId = trackBlue ? 20 : 24;

    for (LLResultTypes.FiducialResult f : fiducials) {
        if (f.getFiducialId() == desiredId) {
            return f.getTargetXDegrees();           // Direct Limelight tx (degrees)
        }
    }
    return Double.NaN;
}

/** Convenience method using current config */
//public double getTargetTx() {
//    return getTargetTx(kTrackBlueTag); // you'll need to make kTrackBlueTag public or add a field
//}
}
