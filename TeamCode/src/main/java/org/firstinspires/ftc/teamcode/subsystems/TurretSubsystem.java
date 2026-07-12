package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class TurretSubsystem extends SubsystemBase {

    private final MotorSubsystem m_turretMotor;
    private final LimelightSubsystem m_vision;
    private final Telemetry m_telemetry;

    public static double kTurretEncoderResolution = 28.0;
    public static boolean kTurretInverted = false;

    // FIX: kAimP=0.2 * max tx (~29.8 deg) = ~6.0, way past kMaxAutoPower=1.0.
    // That meant the controller was effectively bang-bang (full power) for almost
    // the whole range and only proportional in a tiny band near zero -- a classic
    // overshoot/oscillate setup. Lowered so full power is reached only near the
    // actual max expected tx. RE-TUNE ON THE REAL ROBOT -- this is a starting point.
    public static double kAimP = 0.03;
    public static double kFeedForward = 0.08;
    public static double kMaxAutoPower = 1;

    public static double kScanPower = 0.18;
    public static double kLostTargetTimeoutMs = 400;

    // FIX: was `public static boolean kTrackBlueTag`, a STATIC field mutated by an
    // instance method (setTrackBlue). Statics persist across the whole app lifecycle,
    // not per-OpMode-run -- if TeleOp set this to track red mid-match, the NEXT
    // OpMode (e.g. a following Auto) would silently inherit that value unless it
    // explicitly reset it. Made instance-level so each OpMode run starts clean.
    public boolean kTrackBlueTag = true;

    public static double kDeadbandDeg = 2.0;

    // FIX (addition): soft rotational limits. Nothing previously checked the
    // turret's absolute position against a safe range -- sustained tracking or
    // an errant joystick input could wind it past its mechanical stop and damage
    // the wiring. TUNE THESE to your turret's real safe travel range in degrees.
    public static double kMinTurretDeg = -30060.0;
    public static double kMaxTurretDeg = 30060.0;

    private long m_lastSeenTime = 0;
    private boolean m_isTracking = false;

    public TurretSubsystem(HardwareMap hardwareMap, Telemetry telemetry, LimelightSubsystem vision) {
        m_telemetry = telemetry;
        m_vision = vision;

        m_turretMotor = new MotorSubsystem(
                hardwareMap, "turretMotor", kTurretEncoderResolution, kTurretInverted
        );
    }

    public TurretSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, null);
    }

    @Override
    public void periodic() {
        m_telemetry.addLine("Turret:")
                .addData("velRPM", m_turretMotor.getVelocity())
                .addData("posDeg", m_turretMotor.getPositionDegrees())
                .addData("Tracking", m_isTracking)
                .addData("tx", getCurrentTx())
                .addData("power", m_turretMotor.getPower());
    }

    // FIX: now routes through the soft-limit guard, same as automatic tracking.
    public void setPower(double power) {
        m_turretMotor.setRawPower(applySoftLimits(power));
    }

    private double getCurrentTx() {
        return m_vision != null ? m_vision.getTX() : Double.NaN;
    }

    // FIX (addition): shared soft-limit guard used by both manual and auto control.
    private double applySoftLimits(double power) {
        double posDeg = m_turretMotor.getPositionDegrees();
        if (posDeg <= kMinTurretDeg && power < 0) return 0;
        if (posDeg >= kMaxTurretDeg && power > 0) return 0;
        return power;
    }

    public Command manualCommand(java.util.function.DoubleSupplier powerSupplier) {
        // FIX: wrapped supplier so manual joystick input also respects soft limits.
        return m_turretMotor.setPowerCommand(() -> applySoftLimits(powerSupplier.getAsDouble()));
    }

    public Command stopCommand() {
        return m_turretMotor.setPowerCommand(0.0);
    }

    public double computeTrackingPower() {
        if (m_vision == null) return 0;

        double tx = m_vision.getTX();
        if (Double.isNaN(tx)) return 0;
        if (!m_vision.hasTarget()) return 0;

        double power = kAimP * tx;

        // FIX: this deadband logic was BACKWARDS. It added extra feedforward power
        // WHEN THE TARGET WAS ALREADY NEARLY CENTERED (|tx| < deadband), instead of
        // suppressing power near the setpoint. That meant every time the turret got
        // close to dead-center it got an extra kick in whatever direction tx's sign
        // happened to be, overshooting past center, flipping tx's sign, getting
        // kicked again the other way -- a self-sustaining oscillation right around
        // the target. This -- combined with the earlier InstantCommand bug -- is
        // very likely a second, independent cause of the "inching"/jittery tracking.
        //
        // Correct behavior: suppress power inside the deadband (target basically
        // reached, stop fighting it), and only add feedforward help OUTSIDE the
        // deadband where static friction actually needs overcoming.
        if (Math.abs(tx) < kDeadbandDeg) {
            power = 0;
        } else {
            power += Math.signum(tx) * kFeedForward;
        }

        if (power > kMaxAutoPower) power = kMaxAutoPower;
        if (power < -kMaxAutoPower) power = -kMaxAutoPower;

        return applySoftLimits(power);
    }

    public Command lockToTarget() {
        // FIX (from earlier turn, kept here for completeness): was an InstantCommand,
        // which runs its body exactly ONCE then reports itself finished. Under
        // whileHeld(), that meant one stale power write per reschedule -- power on,
        // instantly "done", power off (interrupt cleanup would never even run since
        // it "finished" cleanly), rescheduled next loop -- producing the "inching
        // forward" stutter. FunctionalCommand with isFinished() always false keeps
        // execute() running every loop for the whole button hold.
        return new FunctionalCommand(
                () -> {},
                () -> { m_turretMotor.setRawPower(computeTrackingPower()); },
                interrupted -> m_turretMotor.setRawPower(0.0),
                () -> false,
                this
        );
    }

    public Command autoTrackWithScanCommand() {
        return new FunctionalCommand(
                () -> {
                    m_turretMotor.setRawPower(0.0);
                    m_lastSeenTime = 0;
                    m_isTracking = false;
                },
                () -> {
                    double tx = getCurrentTx();
                    boolean hasTarget = !Double.isNaN(tx) && m_vision != null && m_vision.hasTarget();
                    long now = System.currentTimeMillis();

                    // FIX: this scan/lost-target logic was fully commented out, and
                    // even un-commented it was broken -- m_lastSeenTime was only ever
                    // set to 0 in initialize() and NEVER updated to the current time
                    // when a target WAS actually seen, so "now - m_lastSeenTime >
                    // timeout" could never correctly detect a genuinely lost target.
                    // Implemented properly below: update m_lastSeenTime every time
                    // we see a target, only start scanning after the timeout elapses
                    // since the LAST time we actually saw one.
                    if (hasTarget) {
                        m_isTracking = true;
                        m_lastSeenTime = now;
                    } else if (m_isTracking && m_lastSeenTime > 0 && now - m_lastSeenTime > kLostTargetTimeoutMs) {
                        m_isTracking = false;
                    }

                    if (!m_isTracking) {
                        m_turretMotor.setRawPower(applySoftLimits(kScanPower));
                        return;
                    }

                    double power = computeTrackingPower();
                    m_turretMotor.setRawPower(Double.isNaN(power) ? 0.0 : power);
                },
                interrupted -> m_turretMotor.setRawPower(0.0),
                () -> false,
                this
        );
    }

    // FIX: now sets the instance field instead of a static one.
    public void setTrackBlue(boolean trackBlue) {
        kTrackBlueTag = trackBlue;
    }
}