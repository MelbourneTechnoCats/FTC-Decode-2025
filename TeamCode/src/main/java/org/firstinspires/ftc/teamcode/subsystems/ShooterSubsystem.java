package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ShooterSubsystem extends SubsystemBase {
//    MotorGroup m_motorGroup;//
    private MotorEx m_leftMotor;
    private MotorEx m_rightMotor;
    private MotorGroup m_motorGroup;
    private SimpleServo m_servo;

 public static double kshooterP = 0.001;
 public static double kshooterI = 0;
 public static double kshooterD = 0;
 public static double kshooterA = 0;
 public static double kshooterS = 0.05;
 public static double kshooterV = 0.00018;
 private static final double kshooterGearRatio = 1;
 private static final double kshooterEncoderResolution = 28*kshooterGearRatio;
 private static final double kshooterMaxSpeed = 6000/kshooterGearRatio;
    private Telemetry m_telemetry;
    static final double MIN_ANGLE = 0;
    static final double MAX_ANGLE = 300;

    private PIDController m_pidController;
    private SimpleMotorFeedforward m_ffController;

    private double m_targetVelocity = 0;

    public ShooterSubsystem(final HardwareMap hardwareMap, Telemetry telemetry){
         m_servo = new SimpleServo(hardwareMap, "shooterServo", MIN_ANGLE, MAX_ANGLE);
         m_leftMotor = new MotorEx(hardwareMap, "leftShooterMotor",kshooterEncoderResolution,kshooterMaxSpeed );
         m_rightMotor = new MotorEx(hardwareMap, "rightShooterMotor",kshooterEncoderResolution,kshooterMaxSpeed);
         m_rightMotor.setInverted(true);
         m_motorGroup = new MotorGroup(m_leftMotor, m_rightMotor);
//        m_motorGroup.setRunMode(Motor.RunMode.VelocityControl);
//        m_motorGroup.setVeloCoefficients(kshooterP,kshooterI,kshooterD);
//        m_motorGroup.setFeedforwardCoefficients(kshooterS, kshooterV, kshooterA);
         m_telemetry = telemetry;

         m_pidController = new PIDController(kshooterP, kshooterI, kshooterD);
         m_ffController = new SimpleMotorFeedforward(kshooterS, kshooterV, kshooterA);
    }

    public double getVelocity() { // get velocity in rpm
        double velocity = m_motorGroup.getVelocity(); // in ticks per second
        return velocity * 60 / kshooterEncoderResolution;
    }

    public void periodic() {
        double velocity = getVelocity();
//        double position = m_motorGroup.getCurrentPosition(); // substitutes for angle
//        double revolutions = m_motorGroup.encoder.getRevolutions();
//        double distance = m_motorGroup.encoder.getDistance();
//
        m_telemetry.addData("Actual velocity", velocity);
        m_telemetry.addData("Target velocity", m_targetVelocity);
////        m_telemetry.addData("Position", position);
//        m_telemetry.addData("Revolutions", revolutions);
//        m_telemetry.addData("Distance", distance);
        m_telemetry.update();

        m_pidController.setPID(kshooterP,kshooterI,kshooterD);
        m_ffController = new SimpleMotorFeedforward(kshooterS, kshooterV, kshooterA);

        double power = m_pidController.calculate(getVelocity(), m_targetVelocity) + m_ffController.calculate(m_targetVelocity);
        m_motorGroup.set(Math.max(-1, Math.min(1, power)));
    }


    public void setVelocity(double velocity) {
        m_targetVelocity = velocity;
    }

    public void stop() {
        m_targetVelocity = 0;
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
