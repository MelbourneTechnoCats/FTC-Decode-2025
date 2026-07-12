package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class AutoCommands {
    public ShooterSubsystem shooter;
    public IntakeSubsystem intake;

    public AutoCommands(HardwareMap h, Telemetry t){

        shooter = new ShooterSubsystem(h,t);
        intake = new IntakeSubsystem(h,t);
    }
    public Command shootSequence() {
        return sequential(
                // spin shooter up alone first (inverted, so negative power)
                Commands.instant(() -> shooter.setPower(-1)),
                Commands.waitMs(500),

                // now bring intake in while shooter keeps running
                Commands.instant(() -> {
                    shooter.setPower(-1);
                    intake.setPower(1.0);
                }),
                Commands.waitMs(3000),

                // stop everything at the end
                Commands.instant(() -> {
                    shooter.setPower(0);
                    intake.setPower(0);
                })
        );
    }
    public Command shootWithIntakePulses() {
        return sequential(
                // spin shooter up alone for 1s
                Commands.instant(() -> shooter.setPower(-1)),
                Commands.waitMs(1000),

                // pulse 1
                Commands.instant(() -> { shooter.setPower(-1); intake.setPower(1.0); }),
                Commands.waitMs(150),
                Commands.instant(() -> { shooter.setPower(-1); intake.setPower(0.0); }),
                Commands.waitMs(150),

                // pulse 2
                Commands.instant(() -> { shooter.setPower(-1); intake.setPower(1.0); }),
                Commands.waitMs(150),
                Commands.instant(() -> { shooter.setPower(-1); intake.setPower(0.0); }),
                Commands.waitMs(150),

                // pulse 3
                Commands.instant(() -> { shooter.setPower(-1); intake.setPower(1.0); }),
                Commands.waitMs(150),
                Commands.instant(() -> { shooter.setPower(-1); intake.setPower(0.0); }),
                Commands.waitMs(150),

                // stop everything at the end
                Commands.instant(() -> {
                    shooter.setPower(0);
                    intake.setPower(0);
                })
        );
    }
}
