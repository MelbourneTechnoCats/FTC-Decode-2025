package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Localizer;

/**
 * Drive subsystem that uses mecanum motors and IMU heading only.
 * No odometry wheel position is used; pose (x,y) is always (0,0).
 */
public class NoOdometryDriveSubsystem extends SubsystemBase {

    private final Telemetry m_telemetry;
    public final MecanumDrive m_drive;

    public static final double WIDTH = 18;
    public static final double DEPTH = 18;

    // commanded chassis speeds (unitless, -1..1 style)
    private double m_xSpeed = 0, m_ySpeed = 0, m_rotSpeed = 0;
    // field-relative velocity estimate (computed from commanded speeds)
    private Vector2d m_fieldVelocity = new Vector2d(0, 0);

    private boolean m_fieldCentric = false;

    public NoOdometryDriveSubsystem(final HardwareMap hardwareMap, final Telemetry telemetry) {
        m_telemetry = telemetry;
        // Start at "zero" pose; we do not rely on odometry for position.
        m_drive = new MecanumDrive(hardwareMap,
                new Pose2d(0, 0, 0));
    }

    @Override
    public void periodic() {
        Rotation2d heading = getHeading();
        m_telemetry.addData("Robot heading (deg)", heading.getDegrees());

        // Robot-centric commanded velocities
        Vector2d linearVelocity = new Vector2d(m_xSpeed, m_ySpeed);

        // Convert to field-centric if requested
        if (m_fieldCentric) {
            // rotate by negative heading: robot -> field
            linearVelocity = linearVelocity.rotateBy(-heading.getDegrees());
        }

        // Command Road Runner drive (RR axes swapped)
        m_drive.setDrivePowers(
                new PoseVelocity2d(
                        new com.acmerobotics.roadrunner.Vector2d(
                                linearVelocity.getY(), -linearVelocity.getX()
                        ),
                        m_rotSpeed
                )
        );

        // Expose the commanded "field velocity"
        m_fieldVelocity = new Vector2d(linearVelocity.getX(), linearVelocity.getY());

        m_telemetry.update();
    }

    /**
     * Set drivetrain speeds (unitless, -1..1 style).
     */
    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldCentric) {
        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;
        m_rotSpeed = rotSpeed;
        m_fieldCentric = fieldCentric;
    }

    public Vector2d getFieldVelocity() {
        return m_fieldVelocity;
    }

    /**
     * Get the drivetrain's heading using the current localizer's pose heading.
     * This will typically be IMU-based even if the Localizer is Pinpoint/OTOS/etc.
     */
    public Rotation2d getHeading() {
        Localizer loc = m_drive.localizer;
        double headingRad = loc.getPose().heading.toDouble(); // radians
        return new Rotation2d(headingRad);
    }

    /**
     * Set robot pose. Since we don't track x/y, we only apply heading.
     * x and y components are ignored.
     */
    public void setPose(com.arcrobotics.ftclib.geometry.Pose2d pose) {
        // keep heading from caller, zero x/y
        Pose2d rrPose = new Pose2d(0.0, 0.0, pose.getHeading());
        m_drive.localizer.setPose(rrPose);
    }

    /**
     * Get robot pose. Always returns (0,0,heading).
     */
    public com.arcrobotics.ftclib.geometry.Pose2d getPose() {
        Pose2d rrPose = m_drive.localizer.getPose();
        // ignore x/y, keep heading
        return new com.arcrobotics.ftclib.geometry.Pose2d(
                0.0,
                0.0,
                new Rotation2d(rrPose.heading.toDouble())
        );
    }
}