package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ShooterSubsystem extends SubsystemBase {
    private final MotorSubsystem m;
    private final HoodSubsystem hood;
    private final IntakeSubsystem intake;
    private final LimelightSubsystem limelight;
    private final Telemetry t;

    public static double INCH_TO_M = 0.0254;
    public static double SHOOTER_HEIGHT_M = 0.30;
    public static double TARGET_HEIGHT_M = 0.75;

    public static double FLIGHT_TIME_S = 0.70; // currently unused by the trig-based
    // solver below; kept for future lead compensation

    public static double MAX_LAUNCH_VEL_MPS = 15.0;

    public static double MIN_HOOD_ANGLE_DEG = 10.0;
    public static double MAX_HOOD_ANGLE_DEG = 60.0;

    public static double SHOOTER_WHEEL_RADIUS_M = 0.0254;

    public static double velocityToRpmFactor =
            60.0 / (2.0 * Math.PI * SHOOTER_WHEEL_RADIUS_M);

    public static double kP = 0.004;
    public static double kSP = 0.75; // no longer used directly by aimAndSpinCommand,
    // kept in case you still want a fallback scale
    public static double kI = 0;
    public static double kD = 0;
    public static double kS = 1.0;
    public static double kV = 0.0018;
    public static double kA = 0;

    // FIX (addition): real shooter wheels always lose some exit velocity to
    // ball compression/slip relative to wheel surface speed. This scales the
    // required WHEEL surface speed up so the actual ball exit speed matches
    // the physics solution. Tune from real measured shots (start at 1.0, no
    // correction, and dial down as you observe shots falling short).
    public static double kVelocityEfficiency = 0.9;

    public static double GRAVITY_MPS2 = 9.81;

    // Distance interpolation range for hood-angle selection; tune to your field's
    // actual shot distances.
    public static double MIN_RANGE_M = 1.0;
    public static double MAX_RANGE_M = 4.0;

    public enum DistanceMethod { CAMERA_POSE, ROBOT_POSE, TRIG }
    public static DistanceMethod distanceMethod = DistanceMethod.TRIG;

    // Which AprilTag to aim at. Decoupled from TurretSubsystem's own tracking flag
    // on purpose -- TurretSubsystem.kTrackBlueTag is now an instance field (see fix
    // below), so ShooterSubsystem can't reference it statically anymore anyway.
    public static boolean trackBlueTag = true;

    public ShooterSubsystem(HardwareMap hm, Telemetry telemetry, IntakeSubsystem intake, HoodSubsystem hood, LimelightSubsystem limelight) {
        this.m = new MotorSubsystem(hm, "shooterMotor", 28, false, kP, kI, kD, kS, kV, kA, 0.05);
        this.hood = hood;
        this.intake = intake;
        this.t = telemetry;
        this.limelight = limelight;
    }

    public ShooterSubsystem(HardwareMap hm, Telemetry telemetry) {
        this(hm, telemetry, null, null, null);
    }

    public void setPower(double power) {
        m.setRawPower(power);
    }

    public void setRPM(double rpm) {
        runAtVelocityCommand(rpm).schedule();
    }

    public void setHoodPosition(double ticks) {
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

    /**
     * Picks the distance method configured by `distanceMethod` and reads it
     * for whichever tag `trackBlueTag` points at.
     */
    public double getDistanceToTarget() {
        if (limelight == null) return Double.NaN;

        switch (distanceMethod) {
            case CAMERA_POSE:
                int targetId = trackBlueTag ? LimelightSubsystem.BLUE_TAG_ID : LimelightSubsystem.RED_TAG_ID;
                return limelight.getDistanceCameraPose(targetId);
            case ROBOT_POSE:
                return limelight.getDistanceRobotPose(trackBlueTag);
            case TRIG:
            default:
                return limelight.getDistanceTrig();
        }
    }

    /**
     * Chooses a hood angle for a given distance: steeper for close shots,
     * flatter for long shots.
     */
    public double computeHoodAngleDeg(double distanceM) {
        double clampedDist = Math.max(MIN_RANGE_M, Math.min(MAX_RANGE_M, distanceM));
        double frac = (clampedDist - MIN_RANGE_M) / (MAX_RANGE_M - MIN_RANGE_M);
        return MAX_HOOD_ANGLE_DEG - frac * (MAX_HOOD_ANGLE_DEG - MIN_HOOD_ANGLE_DEG);
    }

    /**
     * Solves required BALL exit velocity (m/s) via projectile motion for a given
     * distance and launch angle. Returns NaN if the angle is too flat to reach
     * the target height at that distance.
     */
    public double computeLaunchVelocityMps(double distanceM, double hoodAngleDeg) {
        double theta = Math.toRadians(hoodAngleDeg);
        double h = TARGET_HEIGHT_M - SHOOTER_HEIGHT_M;

        double denomInner = distanceM * Math.tan(theta) - h;
        if (denomInner <= 0) return Double.NaN;

        double denom = Math.cos(theta) * Math.sqrt((2.0 * denomInner) / GRAVITY_MPS2);
        return distanceM / denom;
    }

    private int angleToTicks(double angleDeg) {
        double clamped = Math.max(MIN_HOOD_ANGLE_DEG, Math.min(MAX_HOOD_ANGLE_DEG, angleDeg));
        double frac = (clamped - MIN_HOOD_ANGLE_DEG) / (MAX_HOOD_ANGLE_DEG - MIN_HOOD_ANGLE_DEG);
        return (int) Math.round(
                HoodSubsystem.MIN_TICKS + frac * (HoodSubsystem.MAX_TICKS - HoodSubsystem.MIN_TICKS)
        );
    }

    public Command aimAndSpinCommand(DriveSubsystem drive) {
        return new InstantCommand(() -> {
            double distance = getDistanceToTarget();
            if (Double.isNaN(distance)) {
                t.addLine("No valid distance -- aborting aim");
                return;
            }

            double hoodAngle = computeHoodAngleDeg(distance);
            double ballExitVel = computeLaunchVelocityMps(distance, hoodAngle);

            if (Double.isNaN(ballExitVel)) {
                t.addLine("Shot geometrically unreachable at this angle/distance");
                return;
            }

            // FIX (addition): account for wheel-to-ball slip. Required wheel
            // SURFACE speed must be higher than the ball's actual exit speed.
            double wheelSurfaceVel = ballExitVel / kVelocityEfficiency;

            if (wheelSurfaceVel > MAX_LAUNCH_VEL_MPS) {
                t.addLine("Shot out of range -- required velocity exceeds MAX_LAUNCH_VEL_MPS");
                return;
            }

            double rpm = wheelSurfaceVel * velocityToRpmFactor;

            t.addData("distance", distance);
            t.addData("hoodAngleDeg", hoodAngle);
            t.addData("ballExitVelMps", ballExitVel);
            t.addData("targetRPM", rpm);

            setHoodPosition(angleToTicks(hoodAngle));
            runAtVelocityCommand(rpm).schedule();
        }, this);
    }

    // FIX: this method previously ignored its own `distanceMeters` parameter
    // entirely and just commanded zero velocity, no matter what distance was
    // passed in. Now it actually solves and applies angle/velocity for that
    // distance, same physics as aimAndSpinCommand.
    public Command shootAtDistance(double distanceMeters) {
        double hoodAngle = computeHoodAngleDeg(distanceMeters);
        double ballExitVel = computeLaunchVelocityMps(distanceMeters, hoodAngle);
        double rpm;
        if (Double.isNaN(ballExitVel)) {
            rpm = 0;
        } else {
            double wheelSurfaceVel = ballExitVel / kVelocityEfficiency;
            rpm = Math.min(wheelSurfaceVel, MAX_LAUNCH_VEL_MPS) * velocityToRpmFactor;
        }

        Command intakePrep = (intake != null) ?
                intake.preShootIntakeCommand() : new InstantCommand(() -> {});

        return new ParallelCommandGroup(
                intakePrep,
                new InstantCommand(() -> setHoodPosition(angleToTicks(hoodAngle))),
                m.setVelocityCommand(rpm)
        );
    }

    @Override
    public void periodic() {
        t.addLine("Shooter:")
                .addData("targetRPM", m.getTargetVelocity())
                .addData("actualRPM", m.getVelocity())
                .addData("atSpeed", isVelocityReached()); // uncommented now that
        // isVelocityReached() div/0 bug is fixed

        if (intake != null) {
            t.addData("Intake Power", intake.getPower());
        }
        if (m.wasVoltageFallbackUsed()) {
            t.addLine("WARNING: shooter voltage sensor unavailable, using fallback voltage");
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