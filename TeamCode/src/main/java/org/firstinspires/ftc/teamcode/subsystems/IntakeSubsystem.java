package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Intake subsystem with two independent motors and no sensors.
 *
 * "Left" and "Right" are just logical names; map them to your actual
 * config names in the constructor.
 */
public class IntakeSubsystem extends SubsystemBase {
    private final MotorEx m_leftMotor;
    private final MotorEx m_rightMotor;

    private final Telemetry m_telemetry;

    public IntakeSubsystem(final HardwareMap hardwareMap, Telemetry telemetry) {
        // TODO: change these strings to match your configuration
        m_leftMotor = new MotorEx(hardwareMap, "leftIntakeMotor");
        m_rightMotor = new MotorEx(hardwareMap, "rightIntakeMotor");
        m_telemetry = telemetry;
    }

    // ---- Raw motor control helpers ----

    public void setLeftPower(double power) {
        m_leftMotor.set(power);
    }

    public void setRightPower(double power) {
        m_rightMotor.set(power);
    }

    public void stopLeft() {
        m_leftMotor.set(0);
    }

    public void stopRight() {
        m_rightMotor.set(0);
    }

    public void stopBoth() {
        m_leftMotor.set(0);
        m_rightMotor.set(0);
    }

    // ---- Simple “both together” helpers (same behavior as old class) ----

    /** Run both intake motors to pull game pieces in (tune directions as needed). */
    public void intakeBoth() {
        m_leftMotor.set(-1.0);
        m_rightMotor.set(-1.0);
    }

    /** Run both intake motors to eject game pieces. */
    public void outtakeBoth() {
        m_leftMotor.set(1.0);
        m_rightMotor.set(1.0);
    }

    // ---- Commands ----

    /** Run both motors inward while this command is scheduled. */
    public Command runCommand() {
        return new StartEndCommand(
                this::intakeBoth,
                this::stopBoth,
                this
        );
    }

    /** Run both motors outward while this command is scheduled. */
    public Command reverseCommand() {
        return new StartEndCommand(
                this::outtakeBoth,
                this::stopBoth,
                this
        );
    }

    /** Stop both motors immediately. */
    public Command stopCommand() {
        return new InstantCommand(this::stopBoth, this);
    }

    /** Run only the left intake while scheduled. */
    public Command runLeftCommand(double power) {
        return new StartEndCommand(
                () -> setLeftPower(power),
                this::stopLeft,
                this
        );
    }

    /** Run only the right intake while scheduled. */
    public Command runRightCommand(double power) {
        return new StartEndCommand(
                () -> setRightPower(power),
                this::stopRight,
                this
        );
    }
}