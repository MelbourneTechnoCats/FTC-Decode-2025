//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.seattlesolvers.solverslib.command.Command;
//import com.seattlesolvers.solverslib.command.CommandOpMode;
//import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
//import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
//import com.seattlesolvers.solverslib.command.SelectCommand;
//import com.seattlesolvers.solverslib.command.WaitCommand;
//import com.seattlesolvers.solverslib.command.WaitUntilCommand;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.IntakeAndSorterSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.SorterSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
//
//import java.util.HashMap;
//
//@Autonomous(name = "Autonomous: Red Alliance, Far Side")
//public class RedFarAutoOpMode extends CommandOpMode {
//    private VisionSubsystem m_visionSubsystem;
//    private DriveSubsystem m_driveSubsystem;
//    private SorterSubsystem m_sorterSubsystem;
//    private IntakeSubsystem m_intakeSubsystem;
//    private IntakeAndSorterSubsystem m_intakeAndSorter;
//    private ShooterSubsystem m_shooterSubsystem;
//
//    private static final double CORNER_X = -48.303871;
//    private static final double CORNER_Y = 63.433975;
//
//    private static final double INITIAL_X = CORNER_X + DriveSubsystem.WIDTH / 2;
//    private static final double INITIAL_Y = CORNER_Y - DriveSubsystem.DEPTH / 2;
//    private static final double INITIAL_HEADING = 0;
//
//    private static final double SHOOT_X = -24;
//    private static final double SHOOT_Y = 12;
//
//    private static final double SHOOT_HEADING =
//            -(Math.PI - Math.atan((Math.abs(CORNER_Y) - Math.abs(SHOOT_Y)) / (72 - Math.abs(SHOOT_X))));
//
//    private static final double SPIKE_START_Y = 24;
//    private static final double SPIKE_END_Y = 48;
//
//    private static final long AUTO_TIMEOUT = 28000;
//
//    private static final long PARK_X = 12;
//    private static final long PARK_Y = 24;
//
//    private static final double TAG_X =
//            -72 + (72 - Math.abs(CORNER_X)) / 2;
//    private static final double TAG_Y =
//            -(Math.abs(CORNER_Y) - (Math.abs(CORNER_Y) - 48) / 2);
//
//    private static final double SHOOT_DISTANCE =
//            Math.sqrt((TAG_X - SHOOT_X) * (TAG_X - SHOOT_X) + (TAG_Y - SHOOT_Y) * (TAG_Y - SHOOT_Y));
//
//    private Command m_shootCommand;
//
//    private static final double SHOOT_ANGLE = 60;
//
//    public Command loadAndShootCommand(double spikeX) { // this assumes that the robot is at the shooting pose
//        return m_driveSubsystem.action2Command(
//                m_driveSubsystem.m_drive.actionBuilder(new Pose2d(SHOOT_X, SHOOT_Y, SHOOT_HEADING))
//                        .turnTo(Math.toRadians(-90))
//                        .strafeTo(new Vector2d(spikeX, SPIKE_START_Y))
//                        .build()
//        ).andThen(
//                m_intakeAndSorter.intakeCommand()
//                        .perpetually()
//                        .raceWith(
//                                m_driveSubsystem.action2Command(
//                                        m_driveSubsystem.m_drive
//                                                .actionBuilder(new Pose2d(spikeX, SPIKE_START_Y, Math.toRadians(-90)))
//                                                .lineToY(SPIKE_END_Y)
//                                                .build()
//                                )
//                        )
//        ).andThen(
//                m_driveSubsystem.action2Command(
//                        m_driveSubsystem.m_drive.actionBuilder(new Pose2d(spikeX, SPIKE_END_Y, Math.toRadians(-90)))
//                                .lineToY(SPIKE_START_Y)
//                                .splineTo(new Vector2d(SHOOT_X, SHOOT_Y), SHOOT_HEADING)
//                                .build()
//                )
//        ).andThen(m_shootCommand);
//    }
//
//    private LiftSubsystem m_liftSubsystem;
//
//    @Override
//    public void initialize() {
//        m_visionSubsystem = new VisionSubsystem(hardwareMap, telemetry);
//        Pose2d initialPose = new Pose2d(INITIAL_X, INITIAL_Y, INITIAL_HEADING);
//        m_driveSubsystem = new DriveSubsystem(hardwareMap, initialPose, telemetry);
//        m_sorterSubsystem = new SorterSubsystem(hardwareMap);
//        m_intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
//        m_intakeAndSorter = new IntakeAndSorterSubsystem(m_intakeSubsystem, m_sorterSubsystem);
//        m_shooterSubsystem = new ShooterSubsystem(hardwareMap, m_intakeAndSorter, telemetry);
//        m_liftSubsystem = new LiftSubsystem(hardwareMap);
//
//        /* initial artifact positions in the sorter */
////        m_sorterSubsystem.occupancy[0] = SorterSubsystem.Colour.PURPLE;
////        m_sorterSubsystem.occupancy[1] = SorterSubsystem.Colour.PURPLE;
////        m_sorterSubsystem.occupancy[2] = SorterSubsystem.Colour.GREEN;
//
//        Command shootPurpleCommand = m_shooterSubsystem.shootCommand(SorterSubsystem.Colour.PURPLE, SHOOT_DISTANCE, SHOOT_ANGLE);
//        Command shootGreenCommand = m_shooterSubsystem.shootCommand(SorterSubsystem.Colour.GREEN, SHOOT_DISTANCE, SHOOT_ANGLE);
//
//        m_shootCommand = new SelectCommand(
//                new HashMap<Object, Command>(){{
//                    put(VisionSubsystem.Motif.GPP, shootGreenCommand.andThen(shootPurpleCommand).andThen(shootPurpleCommand));
//                    put(VisionSubsystem.Motif.PGP, shootPurpleCommand.andThen(shootGreenCommand).andThen(shootPurpleCommand));
//                    put(VisionSubsystem.Motif.PPG, shootPurpleCommand.andThen(shootPurpleCommand).andThen(shootGreenCommand));
//                    put(VisionSubsystem.Motif.NONE, shootPurpleCommand.andThen(shootPurpleCommand).andThen(shootGreenCommand)); // fallback
//                }},
//                m_visionSubsystem::getMotif
//        );
//
//        Command autoCommand =
//                new ParallelCommandGroup(
//                        m_driveSubsystem.action2Command(
//                                m_driveSubsystem.m_drive.actionBuilder(initialPose)
//                                        .strafeTo(new Vector2d(SHOOT_X, SHOOT_Y))
//                                        .turnTo(Math.toRadians(-30))
//                                        .build()
//                        ),
//                        m_intakeAndSorter.getAllColoursCommand(),
//                        m_liftSubsystem.retractCommand()
//                ).andThen(
//                        new ParallelRaceGroup(
//                                new WaitUntilCommand(() -> {
//                                    return m_visionSubsystem.getMotif() != VisionSubsystem.Motif.NONE;
//                                }),
//                                new WaitCommand(1000)
//                        )
//                ).andThen(
//                        m_driveSubsystem.action2Command(
//                                m_driveSubsystem.m_drive.actionBuilder(new Pose2d(SHOOT_X, SHOOT_Y, Math.toRadians(-30)))
//                                        .turnTo(SHOOT_HEADING)
//                                        .build()
//                        )
//                )
//                .andThen(m_shootCommand)
//                .andThen(loadAndShootCommand(-12))
//                .andThen(loadAndShootCommand(12))
//                .andThen(loadAndShootCommand(36))
//                .withTimeout(AUTO_TIMEOUT)
//                .andThen(new SelectCommand(() -> {
//                    com.seattlesolvers.solverslib.geometry.Pose2d pose = m_driveSubsystem.getPose();
//                    return m_driveSubsystem.action2Command(
//                            m_driveSubsystem.m_drive
//                                    .actionBuilder(new Pose2d(pose.getX(), pose.getY(), pose.getHeading()))
//                                    .strafeTo(new Vector2d(PARK_X, PARK_Y))
//                                    .build()
//                    );
//                }));
//
//        schedule(autoCommand);
//    }
//}
