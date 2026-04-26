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
public class NewShooterSubsystem extends SubsystemBase {
    private final MotorSubsystem m_motor;
        public static double kP = 0.004;
    public static double kI = 0;
    public static double kD = 0;
    public static double kS = 1.0;
    public static double kV = 0.0018;
    public static double kA = 0;

    private static final double kshooterGearRatio = 1;
    private static final double kshooterEncoderResolution = 28*kshooterGearRatio;
    private final Telemetry m_telemetry;

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

    // ---- SPLINE-BASED DISTANCE → VELOCITY LUT ----
    /**
     * Sample distances (in meters) to the target.
     * Must be strictly increasing. Tune these in Dashboard.
     */
    public static double[] kLutDistances = new double[] {
            1.0, 1.5, 2.0, 2.5
    };

    /**
     * Corresponding wheel velocities (in RPM) for the above distances.
     * Same length as kLutDistances. Tune these in Dashboard.
     */
    public static double[] kLutVelocities = new double[] {
            2500.0, 2800.0, 3100.0, 3400.0
    };

    /** Spline built from the LUT; lazily created on first use. */
    private SplineInterpolator m_distanceToVelocitySpline = null;
    private boolean m_splineDirty = true;

    // NOTE: available from wheel velocity regression
    private static final double kShooterWheelEfficiency = 0.7; // a
    private static final double kShooterWheelOffset = -557.143 / 60 * 2 * Math.PI; // b (rad/s)

    private static final double kArtifactMass = 84.75 / 1000; // in kg

    private static final double kCoeffA = kShooterWheelInertia * (kShooterWheelEfficiency * kShooterWheelEfficiency - 1);
    private static final double kCoeffB = kShooterWheelInertia * 2 * kShooterWheelEfficiency * kShooterWheelOffset;

    private static final long kRampWaitTime = 400; // extra time to wait for motor to ramp up

    private static double kServoSpeed = 50; // GoBilda Dual Mode Torque servo no-load speed @ 6.0V

//    private static final double kServoZeroAngle = MIN_ANGLE; // servo position where the shooter is pointing 0 deg (outward)
    private static final double kServoGearRatio = (double) 100 / 30;

    private static final double kVelocityTolerance = 0.05; // error margin (proportional to target velocity)

        public NewShooterSubsystem(final HardwareMap hardwareMap, Telemetry telemetry){
        
        m_motor = new MotorSubsystem(
            hardwareMap, "shooterMotor", kshooterEncoderResolution, false,
            kP, kI, kD, kS, kV, kA,
            kVelocityTolerance
        );
        m_telemetry = telemetry;
//        m_intakeAndSorter = intakeAndSorter;
    }
    private double m_goalVelocityMultiplier = 3.525; // TODO: tune this

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

     /**
     * Original physics-based computation preserved for reference.
     */
    public double getGoalVelocityFromDistancePhysics(double angle, double distance) {
        double targetX = distance - (DriveSubsystem.DEPTH * 0.0254) / 2;
        double targetY = kTagY + k_yOffset;
        return getGoalVelocity(targetX, targetY, angle);
    }

    /**
     * New spline-based distance → velocity mapping.
     *
     * @param angle    still accepted for API compatibility but ignored by the spline
     * @param distance distance in meters from shooter to target
     */
    public double getGoalVelocityFromDistance(double angle, double distance) {
        // You can still apply geometry corrections to "distance" here if needed.
        double correctedDistance = distance - (DriveSubsystem.DEPTH * 0.0254) / 2;
        return getSplineVelocityFromDistance(correctedDistance);
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
                    m_intakeAndSorter.loadIntoShooterCommand(colour, strict)
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
                    m_intakeAndSorter.loadIntoShooterCommand(position)
                )
            );
    }

    public Command shootCommandWithVelocity(double velocity, double angle) {
        return runCommand(angle, velocity)
            .raceWith(
                new SequentialCommandGroup(
                    new WaitUntilCommand(this::isVelocityReached),
                    new WaitCommand(kRampWaitTime),
                    m_intakeAndSorter.loadIntoShooterCommand() // any colour
                )
            );
    }

    public void periodic() {
        m_telemetry.addLine("Shooter: ")
            .addData("target", m_motor.getTargetVelocity())
            .addData("actual", m_motor.getVelocity());
        m_telemetry.addData("Shooter velocity reached", isVelocityReached());

        // NOTE: only do this when tuning - comment out once finish
        m_motor.setPIDCoefficients(kP, kI, kD);
        m_motor.setFFCoefficients(kS, kV, kA);
    }

    public boolean isVelocityReached() {
        return m_motor.isVelocityReached();
    }

//    public Command setAngleCommand(double angle) {
//        return m_servo.setAngleCommand(kServoZeroAngle + angle * kServoGearRatio); // TODO: verify direction
//    }
 
    public Command runCommand(double angle, double velocity)
    {
        return new ParallelCommandGroup(
//            setAngleCommand(angle),
            m_motor.setVelocityCommand(velocity)
        );
    }

    public Command stopCommand()
    {
        return new ParallelCommandGroup(
            m_motor.setPowerCommand(0)
        ); // TODO: determine if we want to brake (i.e. set velocity to 0), or we just want to cut off power (as we do right now)
    }

    public double getGoalVelocityMultiplier() {
        return m_goalVelocityMultiplier;
    }

    public void setGoalVelocityMultiplier(double value) {
        m_goalVelocityMultiplier = value;
        // changing multiplier effectively changes mapping; allow callers to rebuild if desired
        m_splineDirty = true;
    }

    public double getLeftVelocity() {
        return m_motor.getVelocity();
    }

    public double getRightVelocity() {
        return m_motor.getVelocity();
    }
    public Command timedRunCommand(double angle, double velocity) {
        return new SequentialCommandGroup(
                // spin up and hold velocity
                runCommand(angle, velocity),
                // wait for fixed duration
                new WaitCommand(500),
                // then stop motors
                stopCommand()
        );
    }

    // ---------------- SPLINE HELPERS ----------------

    /**
     * Mark the spline as invalid so it will be rebuilt next time.
     * Call this if you change kLutDistances or kLutVelocities at runtime.
     */
    public void invalidateSpline() {
        m_splineDirty = true;
    }

    /**
     * Build or rebuild the distance→velocity spline from the LUT arrays.
     * Throws IllegalArgumentException if LUT is invalid.
     */
    private void ensureSpline() {
        if (!m_splineDirty && m_distanceToVelocitySpline != null) {
            return;
        }
        if (kLutDistances == null || kLutVelocities == null
                || kLutDistances.length != kLutVelocities.length
                || kLutDistances.length < 2) {
            throw new IllegalStateException("Shooter LUT must have at least 2 matching distance/velocity points");
        }
        // SplineInterpolator itself checks for strictly increasing x
        m_distanceToVelocitySpline = new SplineInterpolator(kLutDistances, kLutVelocities);
        m_splineDirty = false;
    }

    /**
     * Get goal wheel velocity (RPM) from a given distance using the spline LUT.
     * Optionally scaled by m_goalVelocityMultiplier.
     *
     * @param distance distance in meters from shooter to target
     * @return velocity in RPM
     */
    public double getSplineVelocityFromDistance(double distance) {
        ensureSpline();
        double base = m_distanceToVelocitySpline.interpolate(distance);
        return base * m_goalVelocityMultiplier;
    }
}

