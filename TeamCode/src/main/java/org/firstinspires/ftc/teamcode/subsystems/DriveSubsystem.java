package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class DriveSubsystem extends SubsystemBase {
    private Telemetry m_telemetry;
    private MecanumDrive m_drive;

    private double m_xSpeed = 0, m_ySpeed = 0, m_rotSpeed = 0;
    private boolean m_fieldCentric = false;

    public DriveSubsystem(final HardwareMap hardwareMap, Pose2d pose, final Telemetry telemetry) {
        m_telemetry = telemetry;
        m_drive = new MecanumDrive(hardwareMap, pose);
    }

    @Override
    public void periodic() {
        Rotation2d heading = getHeading();
        m_telemetry.addData("Robot heading (deg)", heading.getDegrees());
        m_telemetry.update();

       Vector2d linearVelocity =
                new Vector2d(m_xSpeed, m_ySpeed);
        if (m_fieldCentric) {
            linearVelocity = linearVelocity.rotateBy(-heading.getDegrees());
        }

        m_drive.setDrivePowers(
                new PoseVelocity2d(
                        new com.acmerobotics.roadrunner.Vector2d(
                                linearVelocity.getY(), -linearVelocity.getX()
                        ), // NOTE: RR has the X and Y axes swapped (rotated by 90deg)
                        m_rotSpeed
                )
        );
        m_drive.updatePoseEstimate();
    }

    /* set velocity to run the drivetrain at (xSpeed and ySpeed in m/s, rotSpeed in rad/s */
    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldCentric) {
        // NOTE: m_xSpeed, m_ySpeed and m_rotSpeed are unitless!
        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;
        m_rotSpeed = rotSpeed;
        m_fieldCentric = fieldCentric;
    }

    /* get the drivetrain's heading */
    public Rotation2d getHeading() {
        double heading = m_drive.localizer.getPose().heading.toDouble(); // in radians
        return new Rotation2d(heading);
    }
}
