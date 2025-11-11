package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.commands.SequentialCommandGroup;

@Config
public class ShooterSubsystem extends SubsystemBase {
//    MotorGroup m_motorGroup;//
    private final MotorSubsystem m_leftMotor;
    private final MotorSubsystem m_rightMotor;
    private final ServoSubsystem m_servo;

    public static double kLeftP = 0.004;
    public static double kLeftI = 0;
    public static double kLeftD = 0;
    public static double kLeftS = 1.0;
    public static double kLeftV = 0.0018;
    public static double kLeftA = 0;

    /* TODO: tune these */
    public static double kRightP = 0.004;
    public static double kRightI = 0;
    public static double kRightD = 0;
    public static double kRightS = 1.0;
    public static double kRightV = 0.0018;
    public static double kRightA = 0;

    private static final double kshooterGearRatio = 1;
    private static final double kshooterEncoderResolution = 28*kshooterGearRatio;
    private final Telemetry m_telemetry;
    static final double MIN_ANGLE = 0;
    static final double MAX_ANGLE = 300;

    private IntakeAndSorterSubsystem m_intakeAndSorter;

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

    private static final long kRampWaitTime = 100; // extra time to wait for motor to ramp up
    private static final long kLoadWaitTime = 100; // extra time to wait for the ball to be shot

    private static double kServoSpeed = 50; // GoBilda Dual Mode Torque servo no-load speed @ 6.0V

    private static final double kServoZeroAngle = MIN_ANGLE; // servo position where the shooter is pointing 0 deg (outward)
    private static final double kServoGearRatio = (double) 100 / 30;

    private static final double kVelocityTolerance = 0.05; // error margin (proportional to target velocity)

    public ShooterSubsystem(final HardwareMap hardwareMap, IntakeAndSorterSubsystem intakeAndSorter, Telemetry telemetry){
        m_servo = new ServoSubsystem(hardwareMap, "shooterServo", kServoSpeed, MIN_ANGLE, MAX_ANGLE);
        m_leftMotor = new MotorSubsystem(
                hardwareMap, "leftShooterMotor", kshooterEncoderResolution, false,
                kLeftP, kLeftI, kLeftD, kLeftS, kLeftV, kLeftA,
                kVelocityTolerance
        );
        m_rightMotor = new MotorSubsystem(
                hardwareMap, "rightShooterMotor", kshooterEncoderResolution, true,
                kRightP, kRightI, kRightD, kRightS, kRightV, kRightA,
                kVelocityTolerance
        );
        m_telemetry = telemetry;
        m_intakeAndSorter = intakeAndSorter;
    }
    private double m_goalVelocityMultiplier = 1.80; // TODO: tune this

    public double getGoalVelocity(double targetX, double targetY, double angle) {
        angle = Math.toRadians(angle);

        double shootingTime = Math.sqrt((2 / kGravity) * ((targetX * Math.tan(angle)) - targetY + kShooterHeight));
        double launchVelocity = targetX / (shootingTime * Math.cos(angle));

        double coeffC = kShooterWheelInertia * kShooterWheelOffset * kShooterWheelOffset + 0.5 * kArtifactMass * launchVelocity * launchVelocity;

        double angVelocity = (-kCoeffB - Math.sqrt(kCoeffB * kCoeffB - 4 * kCoeffA * coeffC)) / (2 * kCoeffA);
        return m_goalVelocityMultiplier * (angVelocity * 60) / (2 * Math.PI);
    }

    public double getGoalVelocityFromRange(double angle, double range){
        double tagX = kCameraX + Math.sqrt(range*range - Math.pow((kTagY - kCameraY), 2));
        double targetX = tagX + k_xOffset;
        double targetY = kTagY + k_yOffset;

        return getGoalVelocity(targetX, targetY, angle);
    }

    public double getGoalVelocityFromDistance(double angle, double distance) {
        double targetX = distance - (DriveSubsystem.DEPTH * 0.0254) / 2;
        double targetY = kTagY + k_yOffset;
        return getGoalVelocity(targetX, targetY, angle);
    }

    public Command shootCommand(int position, double distance, double angle) {
        double velocity = getGoalVelocityFromDistance(angle, distance);
        return shootCommandWithVelocity(position, velocity, angle);
    }

    public Command shootCommand(SorterSubsystem.Colour colour, double distance, double angle, boolean strict) {
        double velocity = getGoalVelocityFromDistance(angle, distance);
        return shootCommandWithVelocity(colour, velocity, angle, strict);
    }

    public Command shootCommand(SorterSubsystem.Colour colour, double distance, double angle) {
        return shootCommand(colour, distance, angle, true);
    }


    public Command shootCommand(double distance, double angle) {
        double velocity = getGoalVelocityFromDistance(angle, distance);
        return shootCommandWithVelocity(velocity, angle);
    }

    public Command shootCommandWithVelocity(SorterSubsystem.Colour colour, double velocity, double angle, boolean strict) {
        return runCommand(angle, velocity)
                .raceWith(
                        new SequentialCommandGroup(
                                new WaitUntilCommand(this::isVelocityReached),
                                new WaitCommand(kRampWaitTime),
                                m_intakeAndSorter.loadIntoShooterCommand(colour, strict),
                                new WaitCommand(kLoadWaitTime)
                        )
                );
    }

    public Command shootCommandWithVelocity(SorterSubsystem.Colour colour, double velocity, double angle) {
        return shootCommandWithVelocity(colour, velocity, angle, true);
    }

    public Command shootCommandWithVelocity(int position, double velocity, double angle) {
        return runCommand(angle, velocity)
                .raceWith(
                        new SequentialCommandGroup(
                                new WaitUntilCommand(this::isVelocityReached),
                                new WaitCommand(kRampWaitTime),
                                m_intakeAndSorter.loadIntoShooterCommand(position),
                                new WaitCommand(kLoadWaitTime)
                        )
                );
    }

    public Command shootCommandWithVelocity(double velocity, double angle) {
        return runCommand(angle, velocity)
                .raceWith(
                        new SequentialCommandGroup(
                                new WaitUntilCommand(this::isVelocityReached),
                                new WaitCommand(kRampWaitTime),
                                m_intakeAndSorter.loadIntoShooterCommand(), // any colour
                                new WaitCommand(kLoadWaitTime)
                        )
                );
    }

    public void periodic() {
        m_telemetry.addLine("Left Shooter: ")
                .addData("target", m_leftMotor.getTargetVelocity())
                .addData("actual", m_leftMotor.getVelocity());
        m_telemetry.addLine("Right Shooter: ")
                .addData("target", m_rightMotor.getTargetVelocity())
                .addData("actual", m_rightMotor.getVelocity());
        m_telemetry.addData("Shooter velocity reached", isVelocityReached());

        // NOTE: only do this when tuning - comment out once finish
        m_leftMotor.setPIDCoefficients(kLeftP, kLeftI, kLeftD);
        m_leftMotor.setFFCoefficients(kLeftS, kLeftV, kLeftA);
        m_rightMotor.setPIDCoefficients(kRightP, kRightI, kRightD);
        m_rightMotor.setFFCoefficients(kRightS, kRightV, kRightA);
    }

    public boolean isVelocityReached() {
        return m_leftMotor.isVelocityReached() && m_rightMotor.isVelocityReached();
    }

    public Command setAngleCommand(double angle) {
        return m_servo.setAngleCommand(kServoZeroAngle + angle * kServoGearRatio); // TODO: verify direction
    }

    public Command runCommand(double angle, double velocity)
    {
        return new ParallelCommandGroup(
                setAngleCommand(angle),
                m_leftMotor.setVelocityCommand(velocity),
                m_rightMotor.setVelocityCommand(velocity)
        );
    }

    public Command stopCommand()
    {
        return new ParallelCommandGroup(
                m_leftMotor.setPowerCommand(0),
                m_rightMotor.setPowerCommand(0)
        ); // TODO: determine if we want to brake (i.e. set velocity to 0), or we just want to cut off power (as we do right now)
    }

    public double getGoalVelocityMultiplier() {
        return m_goalVelocityMultiplier;
    }

    public void setGoalVelocityMultiplier(double value) {
        m_goalVelocityMultiplier = value;
    }
}
