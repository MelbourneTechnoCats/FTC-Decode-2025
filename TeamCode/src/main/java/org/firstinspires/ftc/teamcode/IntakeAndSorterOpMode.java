//package org.firstinspires.ftc.teamcode;
//
//import com.seattlesolvers.solverslib.command.CommandOpMode;
//import com.seattlesolvers.solverslib.command.InstantCommand;
//import com.seattlesolvers.solverslib.command.SelectCommand;
//import com.seattlesolvers.solverslib.gamepad.GamepadEx;
//import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.subsystems.IntakeAndSorterSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.SorterSubsystem;
//
//@TeleOp
//public class IntakeAndSorterOpMode extends CommandOpMode {
//    private IntakeSubsystem m_intakeSubsystem;
//    private SorterSubsystem m_sorterSubsystem;
//    private IntakeAndSorterSubsystem m_intakeAndSorter;
//    private GamepadEx m_gamepad;
//    private boolean m_toIntake = true;
//    private boolean m_retract = true;
//    private int m_position = 0;
//
//    @Override
//    public void initialize() {
//        m_gamepad = new GamepadEx(gamepad1);
//        m_intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
//        m_sorterSubsystem = new SorterSubsystem(hardwareMap);
//        m_intakeAndSorter = new IntakeAndSorterSubsystem(m_intakeSubsystem, m_sorterSubsystem);
//        m_gamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
//                .whenPressed(
//                        new InstantCommand(() -> {
//                            m_toIntake = !m_toIntake;
//                            telemetry.addData("Intake", m_toIntake);
//                            telemetry.addData("Position", m_position);
//                            telemetry.update();
//                        })
//                                .andThen(new SelectCommand(() -> m_intakeAndSorter.setSorterAngleCommand(m_position, m_toIntake)))
//                );
//        m_gamepad.getGamepadButton(GamepadKeys.Button.B)
//                .whenPressed(
//                        new InstantCommand(() -> {
//                            m_position++;
//                            if (m_position > 2)
//                                m_position = 0;
//                            telemetry.addData("Intake", m_toIntake);
//                            telemetry.addData("Position", m_position);
//                            telemetry.update();
//                        })
//                                .andThen(new SelectCommand(() -> m_intakeAndSorter.setSorterAngleCommand(m_position, m_toIntake)))
//                );
//        m_gamepad.getGamepadButton(GamepadKeys.Button.X)
//                .whenPressed(
//                        new InstantCommand(() -> {
//                            m_retract = !m_retract;
//                            telemetry.addData("Retract", m_retract);
//                            telemetry.update();
//                        })
//                                .andThen(new SelectCommand(() -> m_sorterSubsystem.setLeverAngleCommand(m_retract)))
//                );
//    }
//}
