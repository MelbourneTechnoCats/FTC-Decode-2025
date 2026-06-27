package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
 public class ShooterSubsystem extends SubsystemBase {
        private final MotorSubsystem m;
        private final HoodSubsystem hood;
        private final IntakeSubsystem intake;
        private final Telemetry t;

        public static double INCH_TO_M = 0.0254;
        public static double SHOOTER_HEIGHT_M = 0.30;
        public static double TARGET_HEIGHT_M = 0.75;

        public static double FLIGHT_TIME_S = 0.70;
        public static double MAX_LAUNCH_VEL_MPS = 15.0;

        public static double MIN_HOOD_ANGLE_DEG = 10.0;
        public static double MAX_HOOD_ANGLE_DEG = 60.0;

        public static double SHOOTER_WHEEL_RADIUS_M = 0.0254;

        public static double velocityToRpmFactor =
                60.0 / (2.0 * Math.PI * SHOOTER_WHEEL_RADIUS_M);

    public static double kP = 0.004;
    public static double kI = 0;
    public static double kD = 0;
    public static double kS = 1.0;
    public static double kV = 0.0018;
    public static double kA = 0;

    public ShooterSubsystem(HardwareMap hm, Telemetry telemetry, IntakeSubsystem intake, HoodSubsystem hood) {
        this.m = new MotorSubsystem(hm, "shooterMotor", 28, false, kP, kI, kD, kS, kV, kA, 0.05);
        this.hood = hood;
        this.intake = intake;
        this.t = telemetry;
    }

    public ShooterSubsystem(HardwareMap hm, Telemetry telemetry) {
        this(hm, telemetry, null, null);
    }

    public void setPower(double power) {
        m.setRawPower(power);
    }

    public void setRPM(double rpm) {
        runAtVelocityCommand(rpm).schedule();
    }

    public void setHoodPosition(int ticks) {
        if (hood != null) {
            hood.setPosition(ticks);
        }
    }

    public Command runAtVelocityCommand(double rpm) {
        return m.setVelocityCommand(rpm);
    }

    public Command runAtPowerCommand(double power) {
        return m.setPowerCommand(power);
    }

    public Command aimAndSpinCommand(DriveSubsystem drive, double targetXIn, double targetYIn) {
        return new InstantCommand(() -> {
            com.arcrobotics.ftclib.geometry.Pose2d pose = drive.getPose();
            double robotXM = pose.getX() * INCH_TO_M;
            double robotYM = pose.getY() * INCH_TO_M;

            double targetXM = targetXIn * INCH_TO_M;
            double targetYM = targetYIn * INCH_TO_M;

            double dx = targetXM - robotXM;
            double dy = targetYM - robotYM;

            double deltaX = Math.hypot(dx, dy);
            double deltaZ = TARGET_HEIGHT_M - SHOOTER_HEIGHT_M;

            com.arcrobotics.ftclib.geometry.Vector2d fieldVel = drive.getFieldVelocity();
            double dist = Math.hypot(dx, dy);
            double ux = (dist > 1e-6) ? dx / dist : 0.0;
            double uy = (dist > 1e-6) ? dy / dist : 0.0;
            double robotVelocity = fieldVel.getX() * ux + fieldVel.getY() * uy;

            Ballistics.ShooterSolution sol = Ballistics.solve(
                    deltaX,
                    deltaZ,
                    robotVelocity,
                    FLIGHT_TIME_S,
                    velocityToRpmFactor,
                    MIN_HOOD_ANGLE_DEG,
                    MAX_HOOD_ANGLE_DEG,
                    MAX_LAUNCH_VEL_MPS
            );

            if (!sol.valid) {
                t.addData("Ballistics", "INVALID: %s", sol.failureReason);
                return;
            }

            runAtVelocityCommand(sol.rpm).schedule();

            if (hood != null) {
                int hoodTicks = angleToTicks(sol.hoodAngleDegrees);
                hood.setPosition(hoodTicks);
            }

            t.addData("Ballistics", "OK vel=%.2f m/s rpm=%.0f angle=%.1f",
                    sol.launcherVelocity, sol.rpm, sol.hoodAngleDegrees);
        }, this);
    }

    private int angleToTicks(double angleDeg) {
        double clamped = Math.max(MIN_HOOD_ANGLE_DEG, Math.min(MAX_HOOD_ANGLE_DEG, angleDeg));
        double frac = (clamped - MIN_HOOD_ANGLE_DEG) / (MAX_HOOD_ANGLE_DEG - MIN_HOOD_ANGLE_DEG);
        return (int) Math.round(
                HoodSubsystem.MIN_TICKS +
                        frac * (HoodSubsystem.MAX_TICKS - HoodSubsystem.MIN_TICKS)
        );
    }

    public Command shootAtDistance(double distanceMeters) {
        Command intakePrep = (intake != null) ?
                intake.preShootIntakeCommand() : new InstantCommand(() -> {});

        return new ParallelCommandGroup(
                intakePrep,
                m.setVelocityCommand(0)
        );
    }

    @Override
    public void periodic() {
        t.addLine("Shooter:")
                .addData("targetRPM", m.getTargetVelocity())
                .addData("actualRPM", m.getVelocity())
                .addData("atSpeed", isVelocityReached());

        if (intake != null) {
            t.addData("Intake Power", intake.getPower());
        }

        m.setPIDCoefficients(kP, kI, kD);
        m.setFFCoefficients(kS, kV, kA);
    }

    public boolean isVelocityReached() {
        return m.isVelocityReached();
    }

    public Command stop() {
        return new ParallelCommandGroup(
                m.setPowerCommand(0),
                new InstantCommand(() -> {
                    if (hood != null) hood.stop();
                    if (intake != null) intake.stop();
                })
        );
    }

    public HoodSubsystem getHood() {
        return hood;
    }
}