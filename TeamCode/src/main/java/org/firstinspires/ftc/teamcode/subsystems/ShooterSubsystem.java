package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Config
public class ShooterSubsystem extends SubsystemBase {
//    MotorGroup m_motorGroup;//
    private final MotorEx m_leftMotor;
    private final MotorEx m_rightMotor;
    private final MotorGroup m_motorGroup;
    private final SimpleServo m_servo;

 public static double kshooterP = 0.004;
 public static double kshooterI = 0;
 public static double kshooterD = 0;
 public static double kshooterA = 0;
 public static double kshooterS = 1.0;
 public static double kshooterV = 0.0018;
 private static final double kshooterGearRatio = 1;
 private static final double kshooterEncoderResolution = 28*kshooterGearRatio;
 private static final double kshooterMaxSpeed = 6000/kshooterGearRatio;
    private final Telemetry m_telemetry;
    static final double MIN_ANGLE = 0;
    static final double MAX_ANGLE = 300;

    private final PIDController m_pidController;
    private SimpleMotorFeedforward m_ffController;

    private double m_targetVelocity = 0;

    HardwareMap.DeviceMapping<VoltageSensor> m_voltageSensors;



    private static final double kTagY = 61.6/100; /** in m **/

    private static final Position kCameraPosition = VisionSubsystem.kCameraPosition.toUnit(DistanceUnit.METER);
    private static final double kCameraX = kCameraPosition.y;
    private static final double kCameraY = kCameraPosition.z;

    /** constant names are in 2D, but position is in 3D **/
    /** if we change camera position, change the variables as well **/
    /// ////////////
    private static final double k_xOffset = 0;
    private static final double k_yOffset = 0;
    /** todo: get offset done **/
    private static final double kShooterHeight = 30.48/100; /** in m **/
    private static final double kGravity = 9.8; /** in m/s^2 **/

    private static final double kShooterWheelInertia = 6.481E-5;// in kg.m^2

    // NOTE: available from wheel velocity regression
    private static final double kShooterWheelEfficiency = 0.7148; // a
    private static final double kShooterWheelOffset = -33.607; // b (rad/s)

    private static final double kArtifactMass = 84.75 / 1000; // in kg

    private static final double kCoeffA = kShooterWheelInertia * (kShooterWheelEfficiency * kShooterWheelEfficiency - 1);
    private static final double kCoeffB = kShooterWheelInertia * 2 * kShooterWheelEfficiency * kShooterWheelOffset;

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

        m_voltageSensors = hardwareMap.voltageSensor;
    }
    private static final double kGoalVelocityMultiplier = 1.15; // TODO: tune this

    public double getVelocity() { // get velocity in rpm
        double velocity = m_motorGroup.getVelocity(); // in ticks per second
        return velocity * 60 / kshooterEncoderResolution;
    }
    public double getGoalVelocity(double angle, double range){
        double tagX = kCameraX + Math.sqrt(range*range - Math.pow((kTagY - kCameraY), 2));
        double targetX = tagX + k_xOffset;
        double targetY = kTagY + k_yOffset;

        double shootingTime = Math.sqrt((2 / kGravity) * ((targetX * Math.tan(angle)) - targetY + kShooterHeight));
        double launchVelocity = targetX / (shootingTime * Math.cos(angle));

        double coeffC = kShooterWheelInertia * kShooterWheelOffset * kShooterWheelOffset + 0.5 * kArtifactMass * launchVelocity * launchVelocity;

        double angVelocity = (-kCoeffB - Math.sqrt(kCoeffB * kCoeffB - 4 * kCoeffA * coeffC)) / (2 * kCoeffA);
        return kGoalVelocityMultiplier * (angVelocity * 60) / (2 * Math.PI);
    }

    public void periodic() {
        double velocity = getVelocity();
//        double position = m_motorGroup.getCurrentPosition(); // substitutes for angle
//        double revolutions = m_motorGroup.encoder.getRevolutions();
//        double distance = m_motorGroup.encoder.getDistance();

        double voltage = getBatteryVoltage();
        m_telemetry.addData("Battery voltage", voltage);
//
        m_telemetry.addData("Actual velocity", velocity);
        m_telemetry.addData("Target velocity", m_targetVelocity);
////        m_telemetry.addData("Position", position);
//        m_telemetry.addData("Revolutions", revolutions);
//        m_telemetry.addData("Distance", distance);
        m_telemetry.update();

        m_pidController.setPID(kshooterP,kshooterI,kshooterD);
        m_ffController = new SimpleMotorFeedforward(kshooterS, kshooterV, kshooterA);

        double power = (m_pidController.calculate(getVelocity(), m_targetVelocity) + m_ffController.calculate(m_targetVelocity)) / voltage;
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

    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : m_voltageSensors) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}
