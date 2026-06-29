package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
@Config
public class HoodSubsystem extends SubsystemBase {

    private final ServoEx m_servo;

    public static double MIN_POS = 0;
    public static double MAX_POS = 1;

    public static double MIN_TICKS = -10000;
    public static double MAX_TICKS = 10000;

    public static double STEP_POS = 0.01;

    public HoodSubsystem(HardwareMap hardwareMap) {
        m_servo = new ServoEx(
                hardwareMap,
                "hoodServo",
                0,
                1
        );

        double initPos = 0;
        m_servo.set(initPos);
        m_servo.setCachingTolerance(0.05);
    }

    public void setPwm(double pwm) {
        double clamped = Math.max(MIN_POS, Math.min(MAX_POS, pwm));
        m_servo.set(clamped);
    }

    public void setPosition(int ticks) {
//        double clampedTicks = Math.max(MIN_TICKS, Math.min(MAX_TICKS, ticks));
//        double frac = (clampedTicks - MIN_TICKS) / (MAX_TICKS - MIN_TICKS);
//        double pwm = MIN_POS + frac * (MAX_POS - MIN_POS);
        setPwm(ticks);
    }

    public double getCurrentPwm() {
        return m_servo.get();
    }

    public void incrementUp() {
        double current = getCurrentPwm();
        if (Double.isNaN(current)) current = MIN_POS;
        m_servo.set(current + STEP_POS);
    }

    public void incrementDown() {
        double current = getCurrentPwm();
        if (Double.isNaN(current)) current = MIN_POS;
        m_servo.set(current - STEP_POS);
    }

    public void stop() {
    }
}
