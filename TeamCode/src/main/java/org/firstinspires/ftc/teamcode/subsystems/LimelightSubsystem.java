package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

public class LimelightSubsystem extends SubsystemBase {
    public final Limelight3A limelight;
    private final Telemetry telemetry;
    private final DriveSubsystem drive;

    private double tx = 0;
    private double ty = 0;
    private boolean targetVisible;

    private double blueTx = Double.NaN;
    private double redTx = Double.NaN;

    // FIX: cache one LLResult per periodic() cycle. Previously every method
    // (getResult(), getDistance*, periodic()) independently called
    // limelight.getLatestResult(), so two calls in the same loop iteration could
    // return two DIFFERENT camera frames if a new one arrived in between --
    // a real source of jitter in distance/tx readings. All read methods below
    // now use this single cached copy.
    //
    // IMPORTANT: periodic() must actually run every loop for this to stay fresh.
    // In OpModes that don't pump SolversLib's CommandScheduler (e.g. Pedro-based
    // autos), call limelight.periodic() manually once per loop -- see
    // BlueFarAutoOpMode below for an example.
    private LLResult m_cachedResult = null;

    public static final int BLUE_TAG_ID = 20;
    public static final int RED_TAG_ID = 24;

    // Known field positions (meters) of the tracked AprilTags, for getDistanceRobotPose().
    // TODO: fill in with the actual FTC field layout coordinates for this season.
    private static final double BLUE_TAG_X = 0.0;
    private static final double BLUE_TAG_Y = 0.0;
    private static final double RED_TAG_X = 0.0;
    private static final double RED_TAG_Y = 0.0;

    public LimelightSubsystem(HardwareMap hardwareMap, Telemetry telemetry, DriveSubsystem drive) {
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.telemetry = telemetry;
        this.drive = drive;

        limelight.start();
        limelight.setPollRateHz(100);
        setPipeline(2);
    }

    @Override
    public void periodic() {
        m_cachedResult = limelight.getLatestResult();

        blueTx = Double.NaN;
        redTx = Double.NaN;

        if (m_cachedResult != null && m_cachedResult.isValid()) {
            targetVisible = true;
            tx = m_cachedResult.getTx();
            ty = m_cachedResult.getTy();

            List<LLResultTypes.FiducialResult> fiducials = m_cachedResult.getFiducialResults();
            if (fiducials != null) {
                for (LLResultTypes.FiducialResult f : fiducials) {
                    if (f.getFiducialId() == BLUE_TAG_ID) {
                        blueTx = f.getTargetXDegrees();
                        tx=blueTx;
                    } else if (f.getFiducialId() == RED_TAG_ID) {
                        redTx = f.getTargetXDegrees();
                        tx=redTx;
                    }
                }
            }
        } else {
            targetVisible = false;
        }

        telemetry.addData("LL Target", targetVisible ? "VISIBLE" : "NONE");
        telemetry.addData("LL tx", "%.2f", tx);
        if (!Double.isNaN(blueTx)) telemetry.addData("LL Blue Tx", "%.2f", blueTx);
        if (!Double.isNaN(redTx)) telemetry.addData("LL Red Tx", "%.2f", redTx);
    }

    // FIX (removed): the old getDistance() fed degrees straight into Math.tan(),
    // which expects radians -- numerically wrong output. Replaced by the three
    // explicit methods below so callers pick a method on purpose.

    /**
     * Method 1: distance to a SPECIFIC tag via camera-space pose (solvePnP).
     * FIX: previously grabbed fiducials.get(0) with no ID filter -- wrong whenever
     * more than one tag is visible (this robot tracks both blue and red tags
     * simultaneously in periodic()). Now filters by requested tag ID.
     */
    public double getDistanceCameraPose(int targetId) {
        if (m_cachedResult == null || !m_cachedResult.isValid()) return Double.NaN;

        List<LLResultTypes.FiducialResult> fiducials = m_cachedResult.getFiducialResults();
        if (fiducials == null) return Double.NaN;

        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == targetId) {
                Position pos = f.getTargetPoseCameraSpace().getPosition();
                double distance = Math.sqrt(pos.x * pos.x + pos.z * pos.z);
                telemetry.addData("distance (camera pose)", "%.3f", distance);
                return distance;
            }
        }
        return Double.NaN;
    }

    /** Method 2: distance via field-relative botpose vs a known tag field position. */
    public double getDistanceRobotPose(boolean useBlueTag) {
        if (m_cachedResult == null || !m_cachedResult.isValid() || m_cachedResult.getBotpose() == null) {
            return Double.NaN;
        }

        Position robotPos = m_cachedResult.getBotpose().getPosition();
        double targetX = useBlueTag ? BLUE_TAG_X : RED_TAG_X;
        double targetY = useBlueTag ? BLUE_TAG_Y : RED_TAG_Y;

        double dx = targetX - robotPos.x;
        double dy = targetY - robotPos.y;

        double distance = Math.sqrt(dx * dx + dy * dy);
        telemetry.addData("distance (robot pose)", "%.3f", distance);
        return distance;
    }

    /**
     * Method 3: distance via trigonometry using ty and known heights/mount angle.
     * FIX: (mountAngleDeg + ty) is in degrees but was passed directly into
     * Math.tan(), which expects radians. Wrapped in Math.toRadians().
     */
    public double getDistanceTrig() {
        ty = getTY();
        double heightDiffM = 1.175 - 0.35;
        double mountAngleDeg = 20.0;

        double distance = heightDiffM / Math.tan(Math.toRadians(mountAngleDeg + ty));
        telemetry.addData("distance (trig)", "%.3f", distance);
        return distance;
    }

    // FIX: previously called limelight.getLatestResult() twice (no null-guard on
    // the result itself), so this could NPE before the first frame arrives or if
    // the connection drops mid-match. Now uses the cache and null-checks every step.
    public LLResultTypes.FiducialResult getResult() {
        if (m_cachedResult == null) return null;
        List<LLResultTypes.FiducialResult> fiducials = m_cachedResult.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;
        return fiducials.get(0);
    }

    public void setPipeline(int index) {
        limelight.pipelineSwitch(index);
    }

    public double getTX() {
        return tx;
    }

    public double getTY() {
        return ty;
    }

    public boolean hasTarget() {
        return targetVisible;
    }

    public double getHeadingToAprilTag(boolean useBlueTag) {
        double relativeBearing = useBlueTag ? blueTx : redTx;
        if (Double.isNaN(relativeBearing)) return Double.NaN;
        return getRobotHeading() + relativeBearing;
    }

    public double getRobotHeading() {
        if (drive != null) {
            return drive.getHeading().getDegrees();
        }
        if (m_cachedResult != null && m_cachedResult.isValid() && m_cachedResult.getBotpose() != null) {
            return m_cachedResult.getBotpose().getOrientation().getYaw();
        }
        return 0;
    }

    public double getLeadHeadingToAprilTag(boolean useBlueTag, @SuppressWarnings("unused") double projectileSpeed, @SuppressWarnings("unused") double leadScale) {
        return getHeadingToAprilTag(useBlueTag);
    }
}