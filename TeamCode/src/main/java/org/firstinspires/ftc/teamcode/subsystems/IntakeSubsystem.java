package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class IntakeSubsystem extends SubsystemBase {

    private final MotorSubsystem m_intake;
    private final MotorSubsystem m_boost;

    private final Telemetry telemetry;

    // FIX: was 10000.0 -- motor power range the SDK accepts is [-1, 1], so this was
    // being silently clamped to 1.0 internally. It "worked" only by accident, and
    // gave a false impression this constant was tuned. Set to a proper value.
    public static double IN_POWER = 1.0;
    public static double OUT_POWER = -0.7;
    public static double PRE_SHOOT_REVERSE_POWER = -0.6;
    public static double PRE_SHOOT_REVERSE_TIME_MS = 180;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.m_intake = new MotorSubsystem(hardwareMap, "intakeMotor", 28, true);
        this.m_boost = new MotorSubsystem(hardwareMap, "boostMotor", 28, true);
    }

    @Override
    public void periodic() {
    }

    // FIX: previously hardcoded magic number `10` instead of referencing IN_POWER at
    // all. That meant tuning IN_POWER via FTC Dashboard had ZERO effect on normal
    // intake running -- only on the pre-shoot sequence, which used the constant
    // correctly. `10` also gets clamped to `1` internally same as the old IN_POWER
    // bug above -- same accidental-correctness trap.
    public Command runCommand() {
        return new ParallelCommandGroup(
                m_intake.setPowerCommand(() -> IN_POWER),
                m_boost.setPowerCommand(() -> IN_POWER)
        );
    }

    // NOTE: this method is never called from any OpMode -- either dead code left
    // over from an earlier iteration, or a missing button binding. Flagging rather
    // than deleting since I don't know which was intended.
    public Command runBoostAndMotor() {
        return new ParallelCommandGroup(
                m_intake.setPowerCommand(-1.0),
                m_boost.setPowerCommand(-1.0)
        );
    }

    public Command reverseCommand() {
        return new ParallelCommandGroup(
                m_intake.setPowerCommand(() -> OUT_POWER),
                m_boost.setPowerCommand(() -> OUT_POWER)
        );
    }

    public Command stop() {
        return new ParallelCommandGroup(
                m_intake.setPowerCommand(0.0),
                m_boost.setPowerCommand(0.0)
        );
    }

    public Command preShootIntakeCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    m_intake.setRawPower(PRE_SHOOT_REVERSE_POWER);
                    m_boost.setRawPower(PRE_SHOOT_REVERSE_POWER);
                }),
                new WaitCommand((long) PRE_SHOOT_REVERSE_TIME_MS),
                new InstantCommand(() -> {
                    m_intake.setRawPower(IN_POWER);
                    m_boost.setRawPower(IN_POWER);
                })
        );
    }

    public void setPower(double power) {
        m_intake.setRawPower(power);
    }

    public double getPower() {
        return m_intake.getPower();
    }
}