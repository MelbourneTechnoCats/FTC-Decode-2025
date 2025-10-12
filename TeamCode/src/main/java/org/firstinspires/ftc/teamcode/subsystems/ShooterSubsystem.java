package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class ShooterSubsystem extends SubsystemBase {
//    MotorGroup m_motorGroup;
    private MotorEx m_leftMotor;
    private MotorEx m_rightMotor;
    private MotorGroup m_motorGroup;
    private SimpleServo m_servo;


    private Telemetry m_telemetry;
    static final double MIN_ANGLE = 0;
    static final double MAX_ANGLE = 300;


    private static final double kTagY = 61.6/100; /** in m **/

    private static final Position kCameraPosition = VisionSubsystem.kCameraPosition.toUnit(DistanceUnit.METER);
    private static final double kCameraX = kCameraPosition.y;
    private static final double kCameraY = kCameraPosition.z;

    private static final double kShooterWheelDiameter = 7.2/100; /** in m **/
    /** constant names are in 2D, but position is in 3D **/
    /** if we change camera position, change the variables as well **/
    /// ////////////
    private static final double k_xOffset = 0;
    private static final double k_yOffset = 0;
    /** todo: get offset done **/
    private static final double kShooterAngle = Math.toRadians(60); /** in rad **/
    private static final double kShooterHeight = 30.48/100; /** in m **/
    private static final double kGravity = 9.8; /** in m/s^2 **/

    public ShooterSubsystem(final HardwareMap hardwareMap, Telemetry telemetry){
         m_servo = new SimpleServo(hardwareMap, "shooterServo", MIN_ANGLE, MAX_ANGLE);
         m_leftMotor = new MotorEx(hardwareMap, "leftShooterMotor");
         m_rightMotor = new MotorEx(hardwareMap, "rightShooterMotor");
         m_leftMotor.setInverted(true);
         m_motorGroup = new MotorGroup(m_leftMotor, m_rightMotor);
         m_telemetry = telemetry;







    }

    public double getGoalVelocity(double range){
        double tagX = kCameraX + Math.sqrt(range*range - Math.pow((kTagY - kCameraY), 2));
        double targetX = tagX + k_xOffset;
        double targetY = kTagY + k_yOffset;
        double shooterTime = Math.sqrt(2*((targetX*kShooterAngle)-targetY+kShooterHeight)/kGravity);
        double shooterVelocity = targetX*shooterTime/Math.cos(kShooterAngle);
        double shooterRPM = 60*shooterVelocity/(2*Math.PI*kShooterWheelDiameter/2);
        return shooterRPM;



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

    public Command runCommand(double angle, double power)
    {
        return new RunCommand(() -> {
            setAngle(angle);
            setPower(power);
        }, this).whenFinished(this::stop);
    }

    public Command stopCommand()
    {
        return new InstantCommand(this::stop, this);
    }
}
