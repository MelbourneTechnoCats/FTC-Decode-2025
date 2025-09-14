package org.firstinspires.ftc.teamcode.MotorTest.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotorSubsystem extends SubsystemBase {
    private DcMotor _motor;
    private Telemetry _telemetry;

    public MotorSubsystem(DcMotor motor, Telemetry telemetry) {
        _motor = motor;
        _telemetry = telemetry;
    }

    @Override
    public void periodic() {
        _telemetry.addData("Motor power", _motor.getPower());
        _telemetry.update();
    }

    public void runMotor(double power) {
        _motor.setPower(power);
    }

    public void stopMotor() {
        runMotor(0);
    }
}
