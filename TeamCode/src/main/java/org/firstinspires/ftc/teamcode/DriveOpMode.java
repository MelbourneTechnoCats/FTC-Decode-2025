package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeAndSorterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SorterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

//@TeleOp
public abstract class DriveOpMode extends CommandOpMode {
    private GamepadEx m_driveGamepad;
    private DriveSubsystem m_driveSubsystem;
    private IntakeSubsystem m_intakeSubsystem;
    private SorterSubsystem m_sorterSubsystem;
    private VisionSubsystem m_visionSubsystem;
    private IntakeAndSorterSubsystem m_intakeAndSorter;
    private ShooterSubsystem m_shooterSubsystem;
    private boolean m_fieldCentric = false;
    public static double squareInput(double input){

//        if (input >= 0) {
//            output = input*input;
//        }
//        else{
//            output = -input*input;
//        }
//        double output = input*input;
//        if (input < 0) output *=-1;
//        return output;
        return Math.copySign(input*input, input);
    }

    public void initialize(boolean blue) {
        m_driveGamepad = new GamepadEx(gamepad1);
        m_visionSubsystem = new VisionSubsystem(hardwareMap, telemetry);
        m_driveSubsystem = new DriveSubsystem(hardwareMap,new Pose2d(0,0,0), telemetry, m_visionSubsystem);
        m_sorterSubsystem = new SorterSubsystem(hardwareMap);
        m_intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        m_intakeAndSorter = new IntakeAndSorterSubsystem(m_intakeSubsystem, m_sorterSubsystem);
        m_shooterSubsystem = new ShooterSubsystem(hardwareMap, m_intakeAndSorter, telemetry);
        m_driveSubsystem.setDefaultCommand(new RunCommand(
                () -> {
                    double leftX = m_driveGamepad.getLeftX();
                    double leftY = m_driveGamepad.getLeftY();
                    double rightX = m_driveGamepad.getRightX();

                    telemetry.addData("Field-centric drive", m_fieldCentric);
//                    telemetry.update(); // NOTE: telemetry.update() seems to clear telemetry data

                    m_driveSubsystem.drive(
                            squareInput(leftX),
                            squareInput(leftY),
                            -squareInput(rightX),
                            m_fieldCentric
                    );

                    com.arcrobotics.ftclib.geometry.Pose2d pose = m_driveSubsystem.getPose();
                    telemetry.addData("Robot X", pose.getX());
                    telemetry.addData("Robot Y", pose.getY());
                    telemetry.addData("Robot heading", Math.toDegrees(pose.getHeading()));

                    telemetry.addData("Distance to Red Target (m)", m_visionSubsystem.getRedTargetRange());
                    telemetry.addData("Distance to Blue Target (m)", m_visionSubsystem.getBlueTargetRange());

                    telemetry.addData("Intake distance sensor distance (cm)", m_intakeSubsystem.getSensorDistance());
                }, m_driveSubsystem
        ));
        m_sorterSubsystem.setDefaultCommand(new RunCommand(() -> {
                telemetry.addLine("Sorter occupancy: ")
                        .addData("0", m_sorterSubsystem.occupancy[0])
                        .addData("1", m_sorterSubsystem.occupancy[1])
                        .addData("2", m_sorterSubsystem.occupancy[2]);
        }, m_sorterSubsystem));

        m_driveGamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(
                        () -> {
                            m_fieldCentric = !m_fieldCentric;
                        }
                ));

        m_driveGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whileHeld(m_intakeAndSorter.intakeCommand());
        m_driveGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenHeld(m_intakeAndSorter.intakeCommand());

        m_driveGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new SelectCommand(() -> {
                    double range = (blue) ? m_visionSubsystem.getBlueTargetRange() : m_visionSubsystem.getRedTargetRange();
                    if (!Double.isNaN(range)) return m_shooterSubsystem.shootCommand(SorterSubsystem.Colour.GREEN, range, 60); // TODO: adjust angle
                    else return new InstantCommand(() -> {}); // no-op
                }));
        m_driveGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new SelectCommand(() -> {
                    double range = (blue) ? m_visionSubsystem.getBlueTargetRange() : m_visionSubsystem.getRedTargetRange();
                    if (!Double.isNaN(range)) return m_shooterSubsystem.shootCommand(SorterSubsystem.Colour.PURPLE, range, 60); // TODO: adjust angle
                    else return new InstantCommand(() -> {}); // no-op
                }));
        m_driveGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(m_shooterSubsystem.shootCommand(SorterSubsystem.Colour.GREEN, 64, 60));
        m_driveGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(m_shooterSubsystem.shootCommand(SorterSubsystem.Colour.PURPLE, 64, 60));
    }
}
