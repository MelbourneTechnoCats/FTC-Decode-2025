package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.commands.ActionCommand;

public class DriveSubsystem extends SubsystemBase {
    private Telemetry m_telemetry;
    public final MecanumDrive m_drive;

    public static final double WIDTH = 18;
    public static final double DEPTH = 18;

    private double m_xSpeed = 0, m_ySpeed = 0, m_rotSpeed = 0;
    private com.seattlesolvers.solverslib.geometry.Vector2d m_fieldVelocity = new com.seattlesolvers.solverslib.geometry.Vector2d(0, 0);

    private boolean m_fieldCentric = false;

    private VisionSubsystem m_vision;

    public DriveSubsystem(final HardwareMap hardwareMap, Pose2d pose, final Telemetry telemetry) {
        m_telemetry = telemetry;
        m_drive = new MecanumDrive(hardwareMap, pose);
    }

    public DriveSubsystem(final HardwareMap hardwareMap, Pose2d pose, final Telemetry telemetry, final VisionSubsystem vision) {
        m_telemetry = telemetry;
        m_drive = new MecanumDrive(hardwareMap, pose);
        m_vision = vision;
        m_vision.m_poseTrigger.whileActiveContinuous(
                new RunCommand(() -> {
                    setPose(m_vision.getLastPose());
                })  
        );
    }

    public ActionCommand action2Command(Action action)
    {
        return new ActionCommand(action, this);
    }

    @Override
    public void periodic() {
        Rotation2d heading = getHeading();

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
                        ),
                        m_rotSpeed
                )
        );

        PoseVelocity2d rrVel = m_drive.updatePoseEstimate();

        m_fieldVelocity = new com.seattlesolvers.solverslib.geometry.Vector2d(rrVel.linearVel.x, rrVel.linearVel.y);

    }


    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldCentric) {
        // m_xSpeed, m_ySpeed and m_rotSpeed are unitless
        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;
        m_rotSpeed = rotSpeed;
        m_fieldCentric = fieldCentric;
    }
    public com.seattlesolvers.solverslib.geometry.Vector2d getFieldVelocity() {
        return m_fieldVelocity;
    }

    public Rotation2d getHeading() {
        double heading = m_drive.localizer.getPose().heading.toDouble(); // in radians
        return new Rotation2d(heading);
    }


    public void setPose(com.seattlesolvers.solverslib.geometry.Pose2d pose) {
        Pose2d rrPose = new Pose2d(pose.getX(), pose.getY(), pose.getHeading());
        m_drive.localizer.setPose(rrPose);
    }


    public com.seattlesolvers.solverslib.geometry.Pose2d getPose() {
        Pose2d rrPose = m_drive.localizer.getPose();
        return new com.seattlesolvers.solverslib.geometry.Pose2d(
                rrPose.position.x, rrPose.position.y,
                new Rotation2d(rrPose.heading.toDouble())
        );
    }
}
