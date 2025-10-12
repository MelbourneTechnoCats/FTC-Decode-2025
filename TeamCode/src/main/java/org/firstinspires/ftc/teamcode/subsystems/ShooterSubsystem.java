package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterSubsystem extends SubsystemBase {
//    MotorGroup m_motorGroup;//
    private MotorEx m_leftMotor;
    private MotorEx m_rightMotor;
    private MotorGroup m_motorGroup;
    private SimpleServo m_servo;

 private static final double kshooterP = 0;
 private static final double kshooterI = 0;
 private static final double kshooterD = 0;
 private static final double kshooterA = 0;
 private static final double kshooterS = 0;
 private static final double kshooterV = 0;
 private static final double kshooterGearRatio = 3;
 private  static final double kshooterEncoderResolution = 28*kshooterGearRatio;
 private static final double kshooterMaxSpeed = 6000/kshooterGearRatio;
    private Telemetry m_telemetry;
    static final double MIN_ANGLE = 0;
    static final double MAX_ANGLE = 300;
    public ShooterSubsystem(final HardwareMap hardwareMap, Telemetry telemetry){
         m_servo = new SimpleServo(hardwareMap, "shooterServo", MIN_ANGLE, MAX_ANGLE);
         m_leftMotor = new MotorEx(hardwareMap, "leftShooterMotor",kshooterEncoderResolution,kshooterMaxSpeed );
         m_rightMotor = new MotorEx(hardwareMap, "rightShooterMotor",kshooterEncoderResolution,kshooterMaxSpeed);
         m_leftMotor.setInverted(true);
         m_motorGroup = new MotorGroup(m_leftMotor, m_rightMotor);
        m_motorGroup.setRunMode(Motor.RunMode.VelocityControl);
        m_motorGroup.setVeloCoefficients(kshooterP,kshooterI,kshooterD);
        m_motorGroup.setFeedforwardCoefficients(kshooterS, kshooterV, kshooterA);
         m_telemetry = telemetry;


    }
    public void periodic() {
        double velocity = m_motorGroup.getVelocity();
//        double position = m_motorGroup.getCurrentPosition(); // substitutes for angle
//        double revolutions = m_motorGroup.encoder.getRevolutions();
//        double distance = m_motorGroup.encoder.getDistance();
//
        m_telemetry.addData("Velocity", velocity);
////        m_telemetry.addData("Position", position);
//        m_telemetry.addData("Revolutions", revolutions);
//        m_telemetry.addData("Distance", distance);
        m_telemetry.update();

    }


    public void setVelocity(double velocity){
        velocity = velocity/60*kshooterEncoderResolution;
        m_motorGroup.set(velocity);
    }

    public void stop(){
        m_motorGroup.stopMotor();
    }

    public void setAngle(double angle){
        m_servo.turnToAngle(angle);
    }
    public void turnByAngle(double angle){
        m_servo.rotateByAngle(angle);
    }

    public Command runCommand(double angle, double velocity)
    {
        return new RunCommand(() -> {
            setAngle(angle);
            setVelocity(velocity);
        }, this).whenFinished(this::stop);
    }

    public Command stopCommand()
    {
        return new InstantCommand(this::stop, this);
    }
}
