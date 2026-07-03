package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@Config
public class HoodSubsystem extends SubsystemBase {

    private final ServoEx m_servo;

    public static double MIN_POS = 0.32;
    public static double MAX_POS = 0.9;

    public static double MIN_TICKS = 0;
    public static double MAX_TICKS = 1800;

    public static double STEP_POS = 0.01;

    public HoodSubsystem(HardwareMap hardwareMap) {
        m_servo = new ServoEx(hardwareMap, "hoodServo", 0, 1);
        double initPos = 0.32;
        m_servo.set(initPos);
    }

    public void setPwm(double pwm) {
        double clamped = Math.max(MIN_POS, Math.min(MAX_POS, pwm));
        m_servo.set(clamped);
    }

    // FIX: the tick->pwm mapping was commented out, so raw "ticks" (0-1800 range)
    // was passed straight into setPwm(), which clamps to [0.32, 0.9]. Any non-trivial
    // angle command was silently slamming the hood to MAX_POS. Restored the mapping.
    // Also changed param type int -> double so fractional angle-derived ticks aren't
    // truncated before this method even sees them.
    public void setPosition(double ticks) {
        double clampedTicks = Math.max(MIN_TICKS, Math.min(MAX_TICKS, ticks));
        double frac = (clampedTicks - MIN_TICKS) / (MAX_TICKS - MIN_TICKS);
        double pwm = MIN_POS + frac * (MAX_POS - MIN_POS);
        setPwm(pwm);
    }

    public double getCurrentPwm() {
        return m_servo.get();
    }

    // FIX: incrementUp/Down previously called m_servo.set() directly, bypassing
    // setPwm()'s clamp. Holding DPAD could drive the target past MIN_POS/MAX_POS
    // with no limit -- real risk of over-driving the hood mechanism.
    public void incrementUp() {
        double current = getCurrentPwm();
        if (Double.isNaN(current)) current = MIN_POS;
        setPwm(current + STEP_POS);
    }

    public void incrementDown() {
        double current = getCurrentPwm();
        if (Double.isNaN(current)) current = MIN_POS;
        setPwm(current - STEP_POS);
    }

    public void stop() {
    }
}