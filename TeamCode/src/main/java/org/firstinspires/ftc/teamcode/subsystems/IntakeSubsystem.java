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

/**
 * Simple IntakeSubsystem with a single motor.
 * Supports normal intake/outtake + pre-shoot reverse sequence.
 */
@Config
public class IntakeSubsystem extends SubsystemBase {

    private final MotorSubsystem m_intake;
    private final MotorSubsystem m_boost;

    private final Telemetry telemetry;

    // Tunables
    public static double IN_POWER = 10000.0;
    public static double OUT_POWER = -0.7;
    public static double PRE_SHOOT_REVERSE_POWER = -0.6;
    public static double PRE_SHOOT_REVERSE_TIME_MS = 180;   // time to reverse before shooting

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.m_intake = new MotorSubsystem(hardwareMap, "intakeMotor", 28, true);
        this.m_boost = new MotorSubsystem(hardwareMap, "boostMotor", 28, true);
    }

    @Override
    public void periodic() {
//        telemetry.addData("Intake Power", m_intake.);
    }

    // ====================== Basic Commands ======================

    public Command runCommand() {
        return new ParallelCommandGroup(
                m_intake.setPowerCommand(() ->10),
                m_boost.setPowerCommand(() -> 10)
        );
    }

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

    // ====================== Pre-Shoot Sequence ======================

    /**
     * Prepares intake for shooting: reverses briefly, then runs full speed forward.
     * This command should run in parallel with shooter spin-up.
     */
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

    // Direct access (if needed)
    public void setPower(double power) {
        m_intake.setRawPower(power);
    }

    public double getPower() {
        return m_intake.getPower();
    }
}
