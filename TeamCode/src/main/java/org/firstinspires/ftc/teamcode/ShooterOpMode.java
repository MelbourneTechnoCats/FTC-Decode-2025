//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.RunCommand;
//import com.arcrobotics.ftclib.command.SelectCommand;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.subsystems.IntakeAndSorterSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.NewShooterSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.SorterSubsystem;
//
//@TeleOp
//@Config
//public class ShooterOpMode extends CommandOpMode {
//    private GamepadEx m_shootGamepad;
//    private NewShooterSubsystem m_shooterSubsystem;
//    private SorterSubsystem m_sorterSubsystem;
//    private IntakeSubsystem m_intakeSubsystem;
//    private IntakeAndSorterSubsystem m_intakeAndSorter;
//
//    public static double m_velocity = 1800;
//
//    @Override
//    public void initialize() {
////        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        m_shootGamepad = new GamepadEx(gamepad1);
////        m_sorterSubsystem = new SorterSubsystem(hardwareMap);
//        m_intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
////        m_intakeAndSorter = new IntakeAndSorterSubsystem(m_intakeSubsystem, m_sorterSubsystem);
//        m_shooterSubsystem = new NewShooterSubsystem(hardwareMap, telemetry);
//
//        // Shooter only spins when A is held
//        // Shooter and intake run together while A is held
//        m_shootGamepad.getGamepadButton(GamepadKeys.Button.A)
//                .whenHeld(
//                        new SelectCommand(() ->
//                                m_shooterSubsystem.timedRunCommand(m_velocity, 2000)
//
//
//                                        .raceWith(m_intakeSubsystem.runCommand())
//                        )
//                );
//
//
//        m_intakeAndSorter.setDefaultCommand(new RunCommand(() -> { telemetry.update(); }, m_intakeAndSorter));
//
//        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
//
//    }
//}
