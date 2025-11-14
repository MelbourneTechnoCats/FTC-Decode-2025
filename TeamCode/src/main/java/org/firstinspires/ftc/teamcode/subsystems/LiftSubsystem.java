package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftSubsystem extends SubsystemBase {
    /* Axon MAX MK2 specs */
    private static final double kServoMinAngle = 0;
    private static final double kServoMaxAngle = 180; // https://www.reddit.com/r/FTC/comments/z3ygal/comment/ixoaste/
    private static final double kServoSpeed = 60 / (0.115 / 60 * 360);

    /* TODO: verify positions, gear ratios and directions */

    private static final double kLeftGearRatio = (double) 80 / 12;
    private static final double kRightGearRatio = (double) 80 / 15;

    private static final double kServoPositionPadding = 5;

    private static final double kLeftRetractPosition = kServoMaxAngle - kServoPositionPadding;
    private static final double kRightRetractPosition = kServoMinAngle + kServoPositionPadding;

    private static final double kLiftAngle = (kRightGearRatio - kServoPositionPadding) / Math.max(kLeftGearRatio, kRightGearRatio);
    // maximum lift angle achievable on both sides (to avoid unbalancing)

    private static final double kLeftExtendPosition =
            Math.min(kLeftRetractPosition - kLiftAngle * kLeftGearRatio, kServoPositionPadding);
    private static final double kRightExtendPosition =
            Math.max(kRightRetractPosition + kLiftAngle * kRightGearRatio, kServoMaxAngle - kServoPositionPadding);

    private ServoSubsystem m_leftServo, m_rightServo;

    public LiftSubsystem(final HardwareMap hardwareMap) {
        m_leftServo = new ServoSubsystem(hardwareMap, "leftLiftServo", kServoSpeed, kServoMinAngle, kServoMaxAngle);
        m_rightServo = new ServoSubsystem(hardwareMap, "rightLiftServo", kServoSpeed, kServoMinAngle, kServoMaxAngle);
    }

    public Command retractCommand() {
        return new ParallelCommandGroup(
                m_leftServo.setAngleCommand(kLeftRetractPosition),
                m_rightServo.setAngleCommand(kRightRetractPosition)
        );
    }

    public Command extendCommand() {
        return new ParallelCommandGroup(
                m_leftServo.setAngleCommand(kLeftExtendPosition),
                m_rightServo.setAngleCommand(kRightExtendPosition)
        );
    }
}
