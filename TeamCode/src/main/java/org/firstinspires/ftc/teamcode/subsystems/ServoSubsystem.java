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

public class ServoSubsystem extends SubsystemBase {
    private ServoEx m_servo; // underlying servo object

    private double m_currentPosition = Double.NaN;
    private double m_targetPosition = Double.NaN;
    // positions are initially NaN (dummy/not a number); they are to be initialised by the first setAngle call
    // all positions are in degrees

    private final double m_speed; // servo speed (in degrees per second)

    private ElapsedTime m_timer; // internal timer for keeping track of running time

    public ServoSubsystem(HardwareMap hardwareMap, String name, double speed, double minDegrees, double maxDegrees) { // speed is in RPM
        this(hardwareMap, name, speed, minDegrees, maxDegrees, AngleUnit.DEGREES);
    }

    public ServoSubsystem(HardwareMap hardwareMap, String name, double speed, double minAngle, double maxAngle, AngleUnit unit) {
        m_servo = new SimpleServo(hardwareMap, name, minAngle, maxAngle, unit);
        m_speed = (speed / 60) * 360; // convert RPM to deg/s
        m_timer = new ElapsedTime();
        m_timer.reset(); // start timer
    }

    public double getCurrentPosition() {
        return m_currentPosition;
    }

    public double getTargetPosition() {
        return m_targetPosition;
    }

    private boolean m_moving = false; // set if the servo is "moving"

    public long setAngle(double degrees) {
        return setAngle(degrees, AngleUnit.DEGREES);
    }

    public long setAngle(double angle, AngleUnit unit) { // return the expected wait time in msec
        m_servo.turnToAngle(angle, unit);

        if (unit == AngleUnit.RADIANS) m_targetPosition = Math.toDegrees(angle);
        if (Double.isNaN(m_currentPosition)) m_currentPosition = m_targetPosition; // assume that the servo is at the target already

        long time = (long) Math.ceil(Math.abs(m_currentPosition - m_targetPosition) / m_speed); // round up waiting time
        m_moving = (time != 0);
        return time;
    }

    private double m_lastTimestamp = 0; // timestamp (in sec) of last update

    private static final double kUpdateInterval = 0.001; // position update interval

    @Override
    public void periodic() {
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

    public boolean isMoving() {
        return m_moving;
    }

    public Command setAngleCommand(double degrees) {
        return setAngleCommand(degrees, AngleUnit.DEGREES);
    }

    public Command setAngleCommand(double angle, AngleUnit unit) {
        return new InstantCommand(() -> { setAngle(angle, unit); }, this)
                .andThen(new WaitUntilCommand(() -> { return !m_moving; }));
    }

}
