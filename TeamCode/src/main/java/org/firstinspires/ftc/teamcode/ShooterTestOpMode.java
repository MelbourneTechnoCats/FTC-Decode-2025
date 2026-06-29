//package org;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.seattlesolvers.solverslib.command.CommandOpMode;
//import com.seattlesolvers.solverslib.command.RunCommand;
//import com.seattlesolvers.solverslib.gamepad.GamepadEx;
//import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.NewShooterSubsystem;
//
//@TeleOp(name = "Test: Shooter Motor", group = "Test")
//@Config
//public class ShooterTestOpMode extends CommandOpMode {
//
//    private GamepadEx m_gamepad;
//    private NewShooterSubsystem m_shooter;
//    private IntakeSubsystem m_intakeSubsystem;
//
//    // raw test velocity (RPM at motor shaft)
//    public static double testVelocity = 10000.0;
//    // fixed test angle for servo (deg) – change as needed or ignore if you only care about spin
//    public static double testAngle = 45.0;
//        // Mean counter and velocity list
//        private int tickCounter = 0;
//        private java.util.ArrayList<Double> velocityList = new java.util.ArrayList<>();
//        private double velocitySum = 0.0;
//
//    @Override
//    public void initialize() {
//        m_gamepad = new GamepadEx(gamepad1);
//
//        // Shooter subsystem with single motor, no intake/sorter
//        m_shooter = new NewShooterSubsystem(hardwareMap,   telemetry);
//        m_intakeSubsystem = new IntakeSubsystem(hardwareMap,telemetry);
//
//        // Default command: just keep telemetry updating
//            m_shooter.setDefaultCommand(new RunCommand(() -> {
//                double velocity = m_shooter.getLeftVelocity();
//                tickCounter++;
//                velocitySum += velocity;
//                if (tickCounter % 10 == 0) {
//                    velocityList.add(velocity);
//                    double meanVelocity = velocitySum / tickCounter;
//                    telemetry.addData("Mean velocity (last 10)", meanVelocity);
//                }
//                telemetry.addData("Target velocity", velocity);
//                telemetry.addData("Velocity List Size", velocityList.size());
//                telemetry.update();
//            }, m_shooter));
//
//        // Hold A to run shooter motor at testVelocity (and set angle)
//        m_gamepad.getGamepadButton(GamepadKeys.Button.A)
//                .whileHeld(
//                        m_shooter.runCommand(testAngle, testVelocity)
//                                .raceWith(m_intakeSubsystem.runCommand())
//                );
//
//        // When A is released, motor stops
//        m_gamepad.getGamepadButton(GamepadKeys.Button.A)
//                .whenReleased(m_shooter.stopCommand());
//    }
//}
