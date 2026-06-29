//package org.firstinspires.ftc.teamcode;
//
//import com.seattlesolvers.solverslib.command.CommandOpMode;
//import com.seattlesolvers.solverslib.gamepad.GamepadEx;
//import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
//
//@TeleOp
//public class LiftOpMode extends CommandOpMode {
//    private GamepadEx m_gamepad;
//    private LiftSubsystem m_liftSubsystem;
//
//    @Override
//    public void initialize() {
//        m_gamepad = new GamepadEx(gamepad1);
//        m_liftSubsystem = new LiftSubsystem(hardwareMap);
//
//        m_gamepad.getGamepadButton(GamepadKeys.Button.Y)
//                .whenPressed(m_liftSubsystem.extendCommand());
//        m_gamepad.getGamepadButton(GamepadKeys.Button.A)
//                .whenPressed(m_liftSubsystem.retractCommand());
//    }
//}
