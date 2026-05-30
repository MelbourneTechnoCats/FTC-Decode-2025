package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/**
 * Limelight 3A subsystem for AprilTag tracking using Pipeline 1.
 * Provides target tracking data directly from the Limelight hardware without VisionSubsystem.
 */
public class LimelightSubsystem extends SubsystemBase {
    private final Limelight3A limelight;
    private final Telemetry telemetry;
    private final DriveSubsystem drive;

    private double tx;
    private double ty;
    private boolean targetVisible;
    
    private double blueTx = Double.NaN;
    private double redTx = Double.NaN;

    public LimelightSubsystem(HardwareMap hardwareMap, Telemetry telemetry, DriveSubsystem drive) {
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.telemetry = telemetry;
        this.drive = drive;

        // Basic initialization
        limelight.start();
        limelight.setPollRateHz(100);

        // Default to Pipeline 1 (AprilTag tracking)
        setPipeline(2);
    }

    @Override
    public void periodic() {

        LLResult result = limelight.getLatestResult();

        blueTx = Double.NaN;
        redTx = Double.NaN;

        if (result != null && result.isValid()) {
            targetVisible = true;
            tx = result.getTx();
            ty = result.getTy();
            
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null) {
                for (LLResultTypes.FiducialResult f : fiducials) {
                    if (f.getFiducialId() == 20) { // blue obelisk
                        blueTx = f.getTargetXDegrees();
                    } else if (f.getFiducialId() == 24) { // red obelisk
                        redTx = f.getTargetXDegrees();
                    }
                }
            }
        } else {
            targetVisible = false;
        }

        // Telemetry for debugging
        telemetry.addData("LL Target", targetVisible ? "VISIBLE" : "NONE");
        telemetry.addData("LL tx", "%.2f", tx);
        if (!Double.isNaN(blueTx)) telemetry.addData("LL Blue Tx", "%.2f", blueTx);
        if (!Double.isNaN(redTx)) telemetry.addData("LL Red Tx", "%.2f", redTx);
    }

    /**
     * Switches between pipelines.
     * @param index The pipeline index (0-9).
     */
    public void setPipeline(int index) {
        limelight.pipelineSwitch(index);
    }

    /**
     * @return Horizontal offset from the target (-29.8 to 29.8 degrees).
     */
    public double getTX() {
        return tx;
    }

    /**
     * @return Vertical offset from the target (-24.85 to 24.85 degrees).
     */
    public double getTY() {
        return ty;
    }

    /**
     * @return True if a valid target is currently tracked by the Limelight.
     */
    public boolean hasTarget() {
        return targetVisible;
    }

    /**
     * @param useBlueTag True for blue obelisk, false for red.
     * @return Heading to the target AprilTag in field coordinates (degrees).
     */
    public double getHeadingToAprilTag(boolean useBlueTag) {
        double relativeBearing = useBlueTag ? blueTx : redTx;
        if (Double.isNaN(relativeBearing)) return Double.NaN;

        return getRobotHeading() + relativeBearing;
    }

    /**
     * @return Current robot heading in degrees, from DriveSubsystem or Limelight botpose.
     */
    public double getRobotHeading() {
        if (drive != null) {
            return drive.getHeading().getDegrees();
        }
        
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && result.getBotpose() != null) {
            return result.getBotpose().getOrientation().getYaw();
        }
        return 0;
    }

    /**
     * @param useBlueTag True for blue obelisk, false for red.
     * @param projectileSpeed Speed of the projectile (m/s).
     * @param leadScale Scaling factor for the lead.
     * @return Desired heading to lead the target (degrees).
     */
    public double getLeadHeadingToAprilTag(boolean useBlueTag, @SuppressWarnings("unused") double projectileSpeed, @SuppressWarnings("unused") double leadScale) {
        return getHeadingToAprilTag(useBlueTag);
    }
}
