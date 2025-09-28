package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftSubsystem extends SubsystemBase {
    private ServoEx m_leftServo, m_rightServo;

    public LiftSubsystem(final HardwareMap hardwareMap) {
        m_leftServo = new SimpleServo(hardwareMap, "leftLiftServo", 0, 300);
        m_rightServo = new SimpleServo(hardwareMap, "rightLiftServo", 0, 300);

        setAngle(0);
    }

    public void setAngle(double angle) {
        m_leftServo.turnToAngle(180 + angle);
        m_rightServo.turnToAngle(180 - angle);
    }

    public Command setAngleCommand(double angle) {
//        return new RunCommand(() -> {
//            setAngle(angle);
//        }, this).whenFinished(() -> {
//            setAngle(0);
//        });
        return new InstantCommand(() -> {
            setAngle(angle);
        }, this);
    }
}
