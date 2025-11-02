package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSubsystem extends SubsystemBase {
    private MotorEx _motor;

    private Telemetry _telemetry;

    private SorterSubsystem _sorter;
    private final double BALL_DISTANCE = 10;
    private DistanceSensor _distSensor;


    public IntakeSubsystem(final HardwareMap hardwareMap, SorterSubsystem sorterSubsystem, Telemetry telemetry) {
        _motor = new MotorEx(hardwareMap,"intakeMotor");
        _sorter = sorterSubsystem;
        _telemetry = telemetry;
        _distSensor = hardwareMap.get(DistanceSensor.class, "intakeSensor");
    }

    public void runMotor() {
        _motor.set(1.0);
    }

    public void stopMotor() {
        _motor.set(0);
    }

    public Command runCommand()
    {
        return new StartEndCommand(
                this::runMotor, this::stopMotor,
                this
        );
    }

    public boolean isBallThere(){
        return _distSensor.getDistance(DistanceUnit.CM) < BALL_DISTANCE;

    }
    public Command intakeCommand(){
        return _sorter.feedUnoccupiedCompartmentCommand()
                .andThen(runCommand().interruptOn(this::isBallThere))
                .andThen(_sorter.getColourCommand());
    }


    public void test(){

    }

    public Command stopCommand()
    {
        return new InstantCommand(this::stopMotor, this);
    }
}