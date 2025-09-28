package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class DriveSubsystem extends SubsystemBase {
    private Telemetry m_telemetry;
    private IMU m_imu;
    private MecanumDrive m_drive;

    private double m_xSpeed = 0, m_ySpeed = 0, m_rotSpeed = 0;
    private boolean m_fieldCentric = false;

    public DriveSubsystem(final HardwareMap hardwareMap, Pose2d pose, final Telemetry telemetry) {
        m_telemetry = telemetry;
        m_drive = new MecanumDrive(hardwareMap, pose);
    }

    @Override
    public void periodic() {
//        double heading = m_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//        m_telemetry.addData("Robot heading", heading);
//        m_telemetry.update();

            m_drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(m_xSpeed, m_ySpeed),
                       m_rotSpeed));

    }

    /* set velocity to run the drivetrain at */
    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldCentric) {
        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;
        m_rotSpeed = rotSpeed;
        m_fieldCentric = fieldCentric;
    }

    /* get the drivetrain's heading */
//    public Rotation2d getHeading() {
//        return new Rotation2d(m_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
//    }
}
