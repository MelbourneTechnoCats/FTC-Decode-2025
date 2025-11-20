package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeAndSorterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SorterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.HashMap;

@Autonomous(name = "Autonomous: Near Side (leave only)")
public class NearLeaveAutoOpMode extends CommandOpMode {
    private VisionSubsystem m_visionSubsystem;
    private DriveSubsystem m_driveSubsystem;
    private SorterSubsystem m_sorterSubsystem;
    private IntakeSubsystem m_intakeSubsystem;
    private IntakeAndSorterSubsystem m_intakeAndSorter;
    private ShooterSubsystem m_shooterSubsystem;
    private LiftSubsystem m_liftSubsystem;

    private static final double DISTANCE = 36;

    @Override
    public void initialize() {
        m_visionSubsystem = new VisionSubsystem(hardwareMap, telemetry);
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(180));
        m_driveSubsystem = new DriveSubsystem(hardwareMap, initialPose, telemetry);
        m_sorterSubsystem = new SorterSubsystem(hardwareMap);
        m_intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        m_intakeAndSorter = new IntakeAndSorterSubsystem(m_intakeSubsystem, m_sorterSubsystem);
        m_liftSubsystem = new LiftSubsystem(hardwareMap);
        m_shooterSubsystem = new ShooterSubsystem(hardwareMap, m_intakeAndSorter, telemetry);

        Command autoCommand =
                new ParallelCommandGroup(
                        m_liftSubsystem.retractCommand(),
                        m_driveSubsystem.action2Command(
                                m_driveSubsystem.m_drive.actionBuilder(initialPose)
                                        .strafeTo(new Vector2d(-DISTANCE, 0))
                                        .build()
                        )
                );

        schedule(autoCommand);
    }
}
