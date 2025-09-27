package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterSubsystem extends SubsystemBase {
//    MotorGroup m_motorGroup;
    private MotorEx m_leftMotor;
    private MotorEx m_rightMotor;
    private MotorGroup m_motorGroup;
    private SimpleServo m_servo;


    private Telemetry m_telemetry;
    static final double MIN_ANGLE = 0;
    static final double MAX_ANGLE = 300;
    public ShooterSubsystem(final HardwareMap hardwareMap){
         m_servo = new SimpleServo(hardwareMap, "shooterServo", MIN_ANGLE, MAX_ANGLE);
         m_leftMotor = new MotorEx(hardwareMap, "leftShooterMotor");
         m_rightMotor = new MotorEx(hardwareMap, "rightShooterMotor");
         m_rightMotor.setInverted(true);
         m_motorGroup = new MotorGroup(m_leftMotor, m_rightMotor);



    }
    public void periodic() {
        double velocity = m_motorGroup.getVelocity();
        double position = m_motorGroup.getCurrentPosition(); // substitutes for angle
        double revolutions = m_motorGroup.encoder.getRevolutions();
        double distance = m_motorGroup.encoder.getDistance();

        m_telemetry.addData("Velocity", velocity);
        m_telemetry.addData("Position", position);
        m_telemetry.addData("Revolutions", revolutions);
        m_telemetry.addData("Distance", distance);
        m_telemetry.update();

    }


    public void setPower(double power){
        m_motorGroup.set(power);
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
}
