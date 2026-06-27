package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.function.DoubleSupplier;

/**
 * General-purpose smart motor subsystem built on FTCLib {@link MotorEx}.
 *
 * <p>Features:
 * <ul>
 *   <li>Multiple constructors for easy tuning of PID + feedforward or using defaults</li>
 *   <li>Velocity units are in RPM at the motor shaft (after encoder gearing)</li>
 *   <li>Command-based helpers for:
 *       <ul>
 *         <li>Open-loop power control (constant or supplier-based)</li>
 *         <li>Closed-loop velocity control with PID + feedforward (voltage-compensated)</li>
 *       </ul>
 *   </li>
 *   <li>Velocity targeting API with tolerance check via {@link #isVelocityReached()}</li>
 * </ul>
 *
 * <p><b>Encoder / units:</b>
 * <ul>
 *   <li>{@code encoderResolution} is in counts-per-revolution (CPR) at the motor shaft.</li>
 *   <li>{@link #getVelocity()} returns motor shaft speed in RPM.</li>
 * </ul>
 *
 * <p><b>Usage examples</b> (in an OpMode or another subsystem):
 * <pre>{@code
 * MotorSubsystem shooter = new MotorSubsystem(
 *         hardwareMap, "shooterMotor", 28.0, false,
 *         0.004, 0.0, 0.0,   // kP, kI, kD
 *         1.0, 0.0018, 0.0,  // kS, kV, kA
 *         0.05               // 5% velocity tolerance
 * );
 *
 * // Run at fixed 3000 RPM while held
 * gamepadEx.getGamepadButton(GamepadKeys.Button.A)
 *         .whileHeld(shooter.setVelocityCommand(3000.0));
 *
 * // Simple tank drive side with open-loop power
 * MotorSubsystem leftDrive = new MotorSubsystem(hardwareMap, "leftDrive", 28.0);
 * runCommand = leftDrive.setPowerCommand(() -> -gamepad1.left_stick_y);
 * }</pre>
 */
public class MotorSubsystem extends SubsystemBase {
    private MotorEx m_motor;
    private PIDController m_pidController; // PID (closed-loop) controller
    private SimpleMotorFeedforward m_ffController; // feedforward (open-loop) controller

    private static final double kDefaultKP = 0.0;
    private static final double kDefaultKI = 0.0;
    private static final double kDefaultKD = 0.0;
    private static final double kDefaultKS = 0.0;
    private static final double kDefaultKV = 0.0;
    private static final double kDefaultKA = 0.0;

    private static final double kDefaultVelTolerance = 0.05;

    private double m_velTolerance;

    /**
     * Construct a MotorSubsystem with default PID/FF gains and default velocity tolerance.
     *
     * @param hardwareMap        OpMode HardwareMap
     * @param name               configured motor name
     * @param encoderResolution  encoder counts per revolution (CPR) at the motor shaft
     */
    public MotorSubsystem(
            HardwareMap hardwareMap, String name, double encoderResolution /* in cpr */
    ) {
        this(
                hardwareMap, name, encoderResolution, false,
                kDefaultKP, kDefaultKI, kDefaultKD, kDefaultKS, kDefaultKV, kDefaultKA,
                kDefaultVelTolerance
        );
    }

    /**
     * Construct a MotorSubsystem with default PID/FF gains and default velocity tolerance,
     * allowing motor inversion to be specified.
     *
     * @param hardwareMap        OpMode HardwareMap
     * @param name               configured motor name
     * @param encoderResolution  encoder counts per revolution (CPR)
     * @param inverted           true to invert motor direction
     */
    public MotorSubsystem(
            HardwareMap hardwareMap, String name, double encoderResolution, /* in cpr */ boolean inverted
    ) {
        this(
                hardwareMap, name, encoderResolution, inverted,
                kDefaultKP, kDefaultKI, kDefaultKD, kDefaultKS, kDefaultKV, kDefaultKA,
                kDefaultVelTolerance
        );
    }

    /**
     * Construct a MotorSubsystem with default PID/FF gains and custom velocity tolerance.
     *
     * @param hardwareMap        OpMode HardwareMap
     * @param name               configured motor name
     * @param encoderResolution  encoder counts per revolution (CPR)
     * @param velTolerance       acceptable relative error on velocity (e.g. 0.05 = 5%)
     */
    public MotorSubsystem(
            HardwareMap hardwareMap, String name, double encoderResolution, /* in cpr */
            double velTolerance
    ) {
        this(
                hardwareMap, name, encoderResolution, false,
                kDefaultKP, kDefaultKI, kDefaultKD, kDefaultKS, kDefaultKV, kDefaultKA,
                velTolerance
        );
    }

    /**
     * Construct a MotorSubsystem with default PID/FF gains and custom velocity tolerance,
     * allowing motor inversion.
     *
     * @param hardwareMap        OpMode HardwareMap
     * @param name               configured motor name
     * @param encoderResolution  encoder counts per revolution (CPR)
     * @param inverted           true to invert motor direction
     * @param velTolerance       acceptable relative error on velocity (e.g. 0.05 = 5%)
     */
    public MotorSubsystem(
            HardwareMap hardwareMap, String name, double encoderResolution, /* in cpr */ boolean inverted,
            double velTolerance
    ) {
        this(
                hardwareMap, name, encoderResolution, inverted,
                kDefaultKP, kDefaultKI, kDefaultKD, kDefaultKS, kDefaultKV, kDefaultKA,
                velTolerance
        );
    }

    /**
     * Construct a MotorSubsystem with custom PID/FF gains and default velocity tolerance.
     *
     * @param hardwareMap        OpMode HardwareMap
     * @param name               configured motor name
     * @param encoderResolution  encoder counts per revolution (CPR)
     * @param kP                 PID proportional gain
     * @param kI                 PID integral gain
     * @param kD                 PID derivative gain
     * @param kS                 feedforward static gain
     * @param kV                 feedforward velocity gain
     * @param kA                 feedforward acceleration gain
     */
    public MotorSubsystem(
            HardwareMap hardwareMap, String name, double encoderResolution, /* in cpr */
            double kP, double kI, double kD, double kS, double kV, double kA
    ) {
        this(
                hardwareMap, name, encoderResolution, false,
                kP, kI, kD, kS, kV, kA,
                kDefaultVelTolerance
        );

    }

    /**
     * Construct a MotorSubsystem with custom PID/FF gains, default velocity tolerance,
     * and explicit inversion flag.
     *
     * @param hardwareMap        OpMode HardwareMap
     * @param name               configured motor name
     * @param encoderResolution  encoder counts per revolution (CPR)
     * @param inverted           true to invert motor direction
     * @param kP                 PID proportional gain
     * @param kI                 PID integral gain
     * @param kD                 PID derivative gain
     * @param kS                 feedforward static gain
     * @param kV                 feedforward velocity gain
     * @param kA                 feedforward acceleration gain
     */
    public MotorSubsystem(
            HardwareMap hardwareMap, String name, double encoderResolution, /* in cpr */ boolean inverted,
            double kP, double kI, double kD, double kS, double kV, double kA
    ) {
        this(
                hardwareMap, name, encoderResolution, inverted,
                kP, kI, kD, kS, kV, kA,
                kDefaultVelTolerance
        );

    }

    HardwareMap.DeviceMapping<VoltageSensor> m_voltageSensors;

    private final double m_encoderResolution;

    /**
     * Full constructor: custom PID/FF gains, velocity tolerance, and inversion.
     *
     * @param hardwareMap        OpMode HardwareMap
     * @param name               configured motor name
     * @param encoderResolution  encoder counts per revolution (CPR)
     * @param inverted           true to invert motor direction
     * @param kP                 PID proportional gain
     * @param kI                 PID integral gain
     * @param kD                 PID derivative gain
     * @param kS                 feedforward static gain
     * @param kV                 feedforward velocity gain
     * @param kA                 feedforward acceleration gain
     * @param velTolerance       acceptable relative error on velocity (e.g. 0.05 = 5%)
     */
    public MotorSubsystem(
            HardwareMap hardwareMap, String name, double encoderResolution, /* in cpr */ boolean inverted,
            double kP, double kI, double kD, double kS, double kV, double kA,
            double velTolerance
    ) {
        // max RPM argument is not used because we implement our own velocity controller
        m_motor = new MotorEx(hardwareMap, name, encoderResolution, 0);
        m_motor.setInverted(inverted);
        m_encoderResolution = encoderResolution;

        m_pidController = new PIDController(kP, kI, kD);
        m_ffController = new SimpleMotorFeedforward(kS, kV, kA);

        m_voltageSensors = hardwareMap.voltageSensor;
        m_velTolerance = velTolerance;
    }

    /**
     * Update PID gains at runtime (e.g. from Dashboard).
     *
     * @param kP proportional gain
     * @param kI integral gain
     * @param kD derivative gain
     */
    public void setPIDCoefficients(double kP, double kI, double kD) {
        m_pidController.setPID(kP, kI, kD);
    }

    /**
     * Update feedforward gains at runtime.
     *
     * @param kS static gain
     * @param kV velocity gain
     * @param kA acceleration gain
     */
    public void setFFCoefficients(double kS, double kV, double kA) {
        m_ffController = new SimpleMotorFeedforward(kS, kV, kA);
    }

    /** Target velocity in RPM; NaN when running in raw power mode. */
    private double m_targetVelocity = Double.NaN; // will be NaN unless setVelocityCommand is running

    /**
     * Returns the last commanded target velocity in RPM.
     * If the motor is not currently under velocity control, returns {@code Double.NaN}.
     */
    public double getTargetVelocity() {
        return m_targetVelocity;
    }

    /**
     * Create a command that runs the motor at a fixed open-loop power while scheduled.
     *
     * <p>On start: sets motor power and clears the velocity target.<br>
     * On end: stops the motor.
     *
     * @param power power in [-1, 1]
     * @return a {@link Command} suitable for binding to a button
     */
    public Command setPowerCommand(double power) {
        return new StartEndCommand(
                () -> { m_motor.set(power); m_targetVelocity = Double.NaN; },
                () -> { m_motor.set(0); },
                this
        );
    }

    /**
     * Create a command that continuously sets motor open-loop power from a supplier.
     *
     * <p>On init: clears the velocity target.<br>
     * While executing: calls {@code power.getAsDouble()} and applies it to the motor.<br>
     * On end: stops the motor.<br>
     * This command never finishes on its own.
     *
     * @param power supplier of power in [-1, 1] (e.g. gamepad stick)
     * @return a long-running {@link Command}
     */
    public Command setPowerCommand(DoubleSupplier power) { // will never stop by itself unless interrupted
        return new FunctionalCommand(
                () -> { m_targetVelocity = Double.NaN; },
                () -> { m_motor.set(power.getAsDouble()); },
                (Boolean interrupted) -> { m_motor.set(0); },
                () -> false,
                this
        );
    }

    /**
     * Get the current motor shaft speed in RPM.
     *
     * <p>Uses {@link MotorEx#getCorrectedVelocity()} (ticks per second) and converts to RPM
     * using the configured encoder resolution.
     *
     * @return measured velocity in RPM
     */
    public double getVelocity() {
        return m_motor.getCorrectedVelocity()  * 60 / m_encoderResolution;
    }

    /**
     * Get the minimum positive battery voltage from all reported {@link VoltageSensor}s.
     *
     * <p>Used for simple voltage compensation in the velocity controller.
     *
     * @return battery voltage in volts, or {@link Double#POSITIVE_INFINITY} if none valid
     */
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

    /**
     * Create a command that runs closed-loop velocity control at the requested RPM.
     *
     * <p>Behavior while scheduled:
     * <ul>
     *   <li>On init: stores {@code velocity} in {@link #m_targetVelocity}</li>
     *   <li>Each execute(): computes {@code PID(getVelocity, target) + feedforward(target)}</li>
     *   <li>Scales by battery voltage and clamps final power to [-1, 1]</li>
     *   <li>On end: stops motor and clears target velocity (NaN)</li>
     *   <li>Never finishes on its own</li>
     * </ul>
     *
     * @param velocity target RPM at the motor shaft
     * @return a long-running {@link Command} for scheduler use
     */
    public Command setVelocityCommand(double velocity) {
        return new FunctionalCommand(
                () -> { m_targetVelocity = velocity; },
                () -> {
                    double voltage = getBatteryVoltage();
                    double power = (m_pidController.calculate(getVelocity(), velocity) + m_ffController.calculate(velocity)) / voltage;
                    m_motor.set(Math.max(-1, Math.min(1, power)));
                },
                (Boolean interrupted) -> { m_motor.set(0); m_targetVelocity = Double.NaN; },
                () -> false,
                this
        );
    }

    /**
     * Set the motor inversion flag (changes sign of commanded power/velocity).
     *
     * @param inverted true to invert direction
     */
    public void setInverted(boolean inverted) {
        m_motor.setInverted(inverted);
    }

    /**
     * Immediately set raw motor power and clear any active velocity target.
     *
     * @param power power in [-1, 1]
     */
    public void setRawPower(double power) {
        m_targetVelocity = Double.NaN;
        m_motor.set(power);
    }

    /**
     * Get the relative velocity tolerance used by {@link #isVelocityReached()}.
     *
     * @return tolerance as a fraction of target (e.g. 0.05 = 5%)
     */
    public double getVelocityTolerance() {
        return m_velTolerance;
    }

    /**
     * Set the relative velocity tolerance used by {@link #isVelocityReached()}.
     *
     * @param value tolerance as a fraction of target (e.g. 0.05 = 5%)
     */
    public void setVelocityTolerance(double value) {
        m_velTolerance = value;
    }
    public double getPower(){return m_motor.get();}

    /**
     * Returns true if either:
     * <ul>
     *   <li>No velocity target is active (i.e. motor is in power mode), or</li>
     *   <li>The current velocity is within {@link #m_velTolerance} of the target RPM.</li>
     * </ul>
     *
     * <p>The error check is relative:
     * {@code |target - actual| / target <= tolerance}.
     *
     * @return true when at speed or not in velocity mode; false otherwise
     */
    public boolean isVelocityReached() {
        if (Double.isNaN(m_targetVelocity)) return true; // not being controlled by velocity
        return Math.abs((m_targetVelocity - getVelocity()) / m_targetVelocity) <= m_velTolerance;
    }
}
