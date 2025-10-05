package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@TeleOp
public class DriveOpMode extends CommandOpMode {
    private GamepadEx m_driveGamepad;
    private DriveSubsystem m_driveSubsystem;
    private VisionSubsystem m_visionSubsystem;
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
    @Override
    public void initialize() {
        m_driveGamepad = new GamepadEx(gamepad1);
        m_visionSubsystem = new VisionSubsystem(hardwareMap, telemetry);
        m_driveSubsystem = new DriveSubsystem(hardwareMap,new Pose2d(0,0,0), telemetry, m_visionSubsystem);
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
                }, m_driveSubsystem
        ));

        m_driveGamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(
                        () -> {
                            m_fieldCentric = !m_fieldCentric;
                        }
                ));
    }
}
