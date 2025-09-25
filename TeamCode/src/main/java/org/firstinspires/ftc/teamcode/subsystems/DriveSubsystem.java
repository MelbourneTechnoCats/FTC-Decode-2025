package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveSubsystem extends SubsystemBase {
    private Telemetry m_telemetry;
    private IMU m_imu;
    private MecanumDrive m_drive;

    private double m_xSpeed = 0, m_ySpeed = 0, m_rotSpeed = 0;
    private boolean m_fieldCentric = false;

    public DriveSubsystem(final HardwareMap hardwareMap, final Telemetry telemetry) {
        m_telemetry = telemetry;

        /* instantiate Mecanum drivetrain object from FTCLib */
        m_drive = new MecanumDrive(
                new Motor(hardwareMap, "frontLeftDrive", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "frontRightDrive", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "backLeftDrive", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "backRightDrive", Motor.GoBILDA.RPM_312)
        ); // NOTE: in this exact order

        /* initialise the IMU */
        m_imu = hardwareMap.get(IMU.class, "imu"); // get IMU from robot config
        RevHubOrientationOnRobot imuOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        ); // REV Control Hub orientation on robot
        m_imu.initialize(new IMU.Parameters(imuOrientation));
    }

    @Override
    public void periodic() {
        double heading = m_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        m_telemetry.addData("Robot heading", heading);
        m_telemetry.update();

        if (m_fieldCentric)
            m_drive.driveFieldCentric(m_xSpeed, m_ySpeed, m_rotSpeed, heading);
        else
            m_drive.driveRobotCentric(m_xSpeed, m_ySpeed, m_rotSpeed);
    }

    /* set velocity to run the drivetrain at */
    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldCentric) {
        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;
        m_rotSpeed = rotSpeed;
        m_fieldCentric = fieldCentric;
    }

    /* get the drivetrain's heading */
    public Rotation2d getHeading() {
        return new Rotation2d(m_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }
}
