package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * A subsystem for controlling a single servo with smooth motion and angle tracking.
 *
 * <p>This subsystem manages servo position, interpolating smoothly from the current position to
 * a target position at a specified speed. Internally, it tracks the current and target positions
 * in degrees and automatically computes the expected transit time for motion commands.
 *
 * <p>Both degree-based and radian-based angle setters are provided. The subsystem also offers
 * command-based interfaces for integration with CommandBase-based opmodes.
 */
public class ServoSubsystem extends SubsystemBase {
    private ServoEx m_servo; // underlying servo object

    private double m_currentPosition = Double.NaN;
    private double m_targetPosition = Double.NaN;
    // positions are initially NaN (dummy/not a number); they are to be initialised by the first setAngle call
    // all positions are in degrees

    private final double m_speed; // servo speed (in degrees per second)

    private ElapsedTime m_timer; // internal timer for keeping track of running time

    /**
     * Constructs a new ServoSubsystem with angle limits in degrees.
     *
     * @param hardwareMap the opmode HardwareMap used to obtain the servo instance
     * @param name        the name of the servo in the hardware map
     * @param speed       the servo motion speed in revolutions per minute (RPM)
     * @param minDegrees  the minimum angle limit in degrees
     * @param maxDegrees  the maximum angle limit in degrees
     */
    public ServoSubsystem(HardwareMap hardwareMap, String name, double speed, double minDegrees, double maxDegrees) { // speed is in RPM
        this(hardwareMap, name, speed, minDegrees, maxDegrees, AngleUnit.DEGREES);
    }

    private final double m_minAngle;
    private final double m_maxAngle;

    /**
     * Constructs a new ServoSubsystem with angle limits in the specified unit.
     *
     * @param hardwareMap the opmode HardwareMap used to obtain the servo instance
     * @param name        the name of the servo in the hardware map
     * @param speed       the servo motion speed in revolutions per minute (RPM)
     * @param minAngle    the minimum angle limit (in the unit specified)
     * @param maxAngle    the maximum angle limit (in the unit specified)
     * @param unit        the unit of the angle limits: {@code AngleUnit.DEGREES} or
     *                    {@code AngleUnit.RADIANS}
     */
    public ServoSubsystem(HardwareMap hardwareMap, String name, double speed, double minAngle, double maxAngle, AngleUnit unit) {
        m_servo = new SimpleServo(hardwareMap, name, minAngle, maxAngle, unit);
        m_speed = (speed / 60) * 360; // convert RPM to deg/s

        m_minAngle = (unit == AngleUnit.RADIANS) ? Math.toDegrees(minAngle) : minAngle;
        m_maxAngle = (unit == AngleUnit.RADIANS) ? Math.toDegrees(maxAngle) : maxAngle;
    }

    /**
     * Returns the current servo position in degrees.
     *
     * @return the current position, or {@code Double.NaN} if not yet initialized
     */
    public double getCurrentPosition() {
        return m_currentPosition;
    }

    /**
     * Returns the target servo position in degrees.
     *
     * @return the target position, or {@code Double.NaN} if not yet set
     */
    public double getTargetPosition() {
        return m_targetPosition;
    }

    private boolean m_moving = false; // set if the servo is "moving"

    /**
     * Commands the servo to move to a target angle (in degrees).
     *
     * <p>The servo will smoothly move from its current position to the target position at the
     * configured speed. The method returns the expected time (in milliseconds) for the motion
     * to complete.
     *
     * @param degrees the target angle in degrees, clamped to [minAngle, maxAngle]
     * @return the expected motion time in milliseconds
     */
    public long setAngle(double degrees) {
        return setAngle(degrees, AngleUnit.DEGREES);
    }

    /**
     * Commands the servo to move to a target angle in the specified unit.
     *
     * <p>The servo will smoothly move from its current position to the target position at the
     * configured speed. The method returns the expected time (in milliseconds) for the motion
     * to complete.
     *
     * @param angle the target angle (in the unit specified), clamped to [minAngle, maxAngle]
     * @param unit  the unit of the angle: {@code AngleUnit.DEGREES} or {@code AngleUnit.RADIANS}
     * @return the expected motion time in milliseconds
     */
    public long setAngle(double angle, AngleUnit unit) { // return the expected wait time in msec
        m_servo.turnToAngle(angle, unit);

        if (unit == AngleUnit.RADIANS) angle = Math.toDegrees(angle);
        if (angle < m_minAngle) angle = m_minAngle;
        else if (angle > m_maxAngle) angle = m_maxAngle;

        m_targetPosition = angle;
        if (Double.isNaN(m_currentPosition)) m_currentPosition = m_targetPosition; // assume that the servo is at the target already

        long time = (long) Math.ceil(Math.abs(m_currentPosition - m_targetPosition) / m_speed); // round up waiting time
        m_moving = (time != 0);
        return time;
    }

    private double m_lastTimestamp = 0; // timestamp (in sec) of last update

    private static final double kUpdateInterval = 0.001; // position update interval

    /**
     * Periodic update called by the scheduler. Updates the current servo position estimate
     * based on elapsed time and configured speed, marking the servo as idle once the target is
     * reached.
     */
    @Override
    public void periodic() {
        if (m_timer == null) {
            m_timer = new ElapsedTime();
            m_timer.reset(); // start timer
        }

        double currentTimestamp = m_timer.seconds();
        double elapsedTime = Math.abs(currentTimestamp - m_lastTimestamp); // the abs is there just to be safe
        if (elapsedTime >= kUpdateInterval) {
            if (!Double.isNaN(m_currentPosition) && m_moving) {
                double delta = m_speed * elapsedTime; // change in angle
                if (Math.abs(m_currentPosition - m_targetPosition) <= delta) { // very close to target
                    m_currentPosition = m_targetPosition;
                    m_moving = false;
                }
                else if (m_currentPosition < m_targetPosition) m_currentPosition += delta;
                else m_currentPosition -= delta;
            }
            m_lastTimestamp = currentTimestamp;
        }
    }

    /**
     * Returns whether the servo is currently moving toward its target position.
     *
     * @return true if the servo is in motion; false if it has reached the target or is idle
     */
    public boolean isMoving() {
        return m_moving;
    }

    /**
     * Returns a Command that moves the servo to a target angle (in degrees) and waits until
     * motion completes.
     *
     * @param degrees the target angle in degrees
     * @return a Command that executes the motion and waits for completion
     */
    public Command setAngleCommand(double degrees) {
        return setAngleCommand(degrees, AngleUnit.DEGREES);
    }

    /**
     * Returns a Command that moves the servo to a target angle in the specified unit and waits
     * until motion completes.
     *
     * @param angle the target angle (in the unit specified)
     * @param unit  the unit of the angle: {@code AngleUnit.DEGREES} or {@code AngleUnit.RADIANS}
     * @return a Command that executes the motion and waits for completion
     */
    public Command setAngleCommand(double angle, AngleUnit unit) {
        return new InstantCommand(() -> { setAngle(angle, unit); }, this)
                .andThen(new WaitUntilCommand(() -> { return !m_moving; }));
    }

}
