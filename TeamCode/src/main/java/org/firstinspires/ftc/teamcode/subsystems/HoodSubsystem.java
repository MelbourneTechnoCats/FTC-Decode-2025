package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Hood subsystem backed by a servo instead of a motor.
 *
 * Uses ServoSubsystem to move the hood to a desired angle (degrees).
 */
@Config
public class HoodSubsystem {

    private final ServoSubsystem m_servo;

    // Hood physical range in degrees (tune these!)
    public static double MIN_HOOD_ANGLE_DEG = 0.0;
    public static double MAX_HOOD_ANGLE_DEG = 60.0;

    // Servo speed in RPM (from datasheet; adjust if needed)
    public static double SERVO_SPEED_RPM = 50.0;

    // Step size for manual nudging
    public static double STEP_DEG = 1.0;

    public HoodSubsystem(HardwareMap hardwareMap) {
        // "hoodServo" must be configured in the RC config as a servo
        m_servo = new ServoSubsystem(
                hardwareMap,
                "hoodServo",
                SERVO_SPEED_RPM,
                MIN_HOOD_ANGLE_DEG,
                MAX_HOOD_ANGLE_DEG,
                AngleUnit.DEGREES
        );
    }

    /** Move hood to a specific angle in degrees (clamped to [MIN, MAX]). */
    public void setAngle(double angleDeg) {
        m_servo.setAngle(angleDeg, AngleUnit.DEGREES);
    }

    /** Current estimated hood angle in degrees. */
    public double getCurrentAngle() {
        return m_servo.getCurrentPosition();
    }

    /** Nudge hood up by STEP_DEG. */
    public void incrementUp() {
        double current = Double.isNaN(getCurrentAngle()) ? MIN_HOOD_ANGLE_DEG : getCurrentAngle();
        setAngle(current + STEP_DEG);
    }

    /** Nudge hood down by STEP_DEG. */
    public void incrementDown() {
        double current = Double.isNaN(getCurrentAngle()) ? MIN_HOOD_ANGLE_DEG : getCurrentAngle();
        setAngle(current - STEP_DEG);
    }

    /** Stop is effectively a no-op for positional servo, but kept for API compatibility. */
    public void stop() {
        // nothing to do; positional servos hold last command
    }
}