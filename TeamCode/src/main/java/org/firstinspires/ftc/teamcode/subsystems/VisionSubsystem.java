package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class VisionSubsystem extends SubsystemBase {
    /* camera pose on robot - used for robot pose determination */
    private static final Position kCameraPosition = new Position(DistanceUnit.CM,
            19, 20.5, 34, 0);
    private static final YawPitchRollAngles kCameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    private AprilTagProcessor m_tagProcessor;
    private VisionPortal m_visionPortal;

    private double m_xPosition, m_yPosition, m_heading;

    private Telemetry m_telemetry;
    private AprilTagPoseFtc m_redTargetPose, m_blueTargetPose;

    public class PoseTrigger extends Trigger {
        boolean m_update = false; // true if there's a pose update in this iteration

        @Override
        public boolean get() {
            return m_update;
        }
    }
    public final PoseTrigger m_poseTrigger = new PoseTrigger();

    public VisionSubsystem(final HardwareMap hardwareMap, final Telemetry telemetry) {
        m_telemetry = telemetry;

        /* create the AprilTag processor */
        m_tagProcessor = new AprilTagProcessor.Builder()
                .setCameraPose(kCameraPosition, kCameraOrientation)
                .build();

        m_visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .enableLiveView(true)
                .addProcessor(m_tagProcessor)
                .build();
    }

    @Override
    public void periodic() {
        m_blueTargetPose = null;
        m_redTargetPose = null;
        m_poseTrigger.m_update = false;

        List<AprilTagDetection> detections = m_tagProcessor.getDetections();

        double meanX = 0, meanY = 0, meanEndX = 0, meanEndY = 0;
        int numPoints = 0;
        for (AprilTagDetection detection : detections) {
            if (!detection.metadata.name.contains("Obelisk")) {
                Position pos = detection.robotPose.getPosition();
                meanX += pos.x; meanY += pos.y;
                Vector2d vec = new Vector2d(144, 0).rotateBy(
                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)
                );
                meanEndX += pos.x + vec.getX();
                meanEndY += pos.y + vec.getY();
                numPoints++;
                if (detection.metadata.id == 20){
                    m_blueTargetPose = detection.ftcPose;
                }
                else if (detection.metadata.id == 24)
                {
                    m_redTargetPose = detection.ftcPose;
                }
            }
        }

//        m_telemetry.addData("Number of localisation tags", numPoints);

        if (numPoints > 0) {
            meanX /= numPoints; meanY /= numPoints;
            meanEndX /= numPoints; meanEndY /= numPoints;

            m_xPosition = meanX; m_yPosition = meanY;

            Vector2d vec = new Vector2d(meanEndX - meanX, meanEndY - meanY);
            m_heading = Math.toDegrees(vec.angle());

//            m_telemetry.addData("Vision X", m_xPosition);
//            m_telemetry.addData("Vision Y", m_yPosition);
//            m_telemetry.addData("Vision heading", m_heading);

            m_poseTrigger.m_update = true;
        }

//        m_telemetry.update();
    }

    public Pose2d getLastPose() {
        return new Pose2d(m_xPosition, m_yPosition, new Rotation2d(Math.toRadians(m_heading)));
    }

    public AprilTagPoseFtc getRedTargetPose()
    {
        return m_redTargetPose;
    }

    public AprilTagPoseFtc getBlueTargetPose()
    {
        return m_blueTargetPose;
    }
}
