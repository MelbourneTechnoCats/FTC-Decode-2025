package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    private MotorEx _motor;
    private Telemetry _telemetry;

    public IntakeSubsystem(final HardwareMap hardwareMap, Telemetry telemetry) {
        _motor = new MotorEx(hardwareMap,"intakeMotor");
        _telemetry = telemetry;
    }

    public void runMotor() {
        _motor.set(1.0);
    }

    public void stopMotor() {
        _motor.set(0);
    }

    public Command runCommand()
    {
        return new RunCommand(() -> {
            runMotor();
        }, this).whenFinished(this::stopMotor);
    }

    public Command stopCommand()
    {
        return new InstantCommand(this::stopMotor, this);
    }
}