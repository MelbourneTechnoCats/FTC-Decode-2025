package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Autonomous(name = "blue far auto", group = "Autonomous")
@Configurable
public class BlueFarAutoOpMode extends LinearOpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private LimelightSubsystem limelight;
    private HoodSubsystem hood;
    private Paths paths;

    @Override
    public void runOpMode() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // FIX: follower was previously built TWICE -- once here, once again right
        // before waitForStart(). paths.MainChain was built against the FIRST
        // follower object, which then got discarded when the second was assigned,
        // potentially detaching the path from the follower actually driving.
        // Now built exactly once.
        follower = Constants.createFollower(hardwareMap);

        // FIX: `1-Math.toRadians(90)` (~-0.57 rad) looked like a typo for
        // `-Math.toRadians(90)` (-90 deg exactly). Flagging and correcting --
        // double check this matches your intended starting heading.
        follower.setStartingPose(new Pose(84.101, 3.74, -Math.toRadians(90)));

        // FIX: construction order corrected so ShooterSubsystem receives REAL
        // intake/hood/limelight references instead of null. Previously:
        //   shooter = new ShooterSubsystem(hardwareMap, telemetry, intake, hood, limelight);
        //   intake = new IntakeSubsystem(...);   // assigned AFTER shooter needed it
        //   hood = new HoodSubsystem(...);       // assigned AFTER shooter needed it
        // and `limelight` was NEVER constructed at all (declared, never `new`'d).
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        hood = new HoodSubsystem(hardwareMap);
        limelight = new LimelightSubsystem(hardwareMap, telemetry, null); // no DriveSubsystem
        // in this Pedro-based auto;
        // LimelightSubsystem falls
        // back to botpose heading
        shooter = new ShooterSubsystem(hardwareMap, telemetry, intake, hood, limelight);

        // FIX: paths now built AFTER the (single) final follower + starting pose
        // are set, so the path is bound to the follower that's actually running.
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        Scheduler.reset();

        waitForStart();

        schedule(autoRoutine());
        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            // FIX (addition): LimelightSubsystem.periodic() is normally driven by
            // SolversLib's CommandScheduler via register(...), but this Pedro-based
            // auto never registers subsystems and never runs that scheduler --
            // only Pedro's own ivy Scheduler.execute() above. Without this manual
            // pump, LimelightSubsystem's cached LLResult (see its caching fix)
            // would stay null forever and every distance read below would return
            // NaN the whole match.
            limelight.periodic();

            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("distance to target", limelight.getDistanceTrig());
            telemetry.update();

            panelsTelemetry.debug("x", follower.getPose().getX());
            panelsTelemetry.debug("y", follower.getPose().getY());
            panelsTelemetry.update(telemetry);
        }

        // FIX (addition): ensure motors are off when auto ends -- previously
        // nothing stopped the shooter/intake at the end of the routine or loop,
        // so they'd hold their last commanded power indefinitely.
        shooter.setPower(0);
        intake.setPower(0);
    }

    public static class Paths {
        public PathChain MainChain;

        public Paths(Follower follower) {
            MainChain = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(84.101, 3.742),
                                    new Pose(46.491, 100.150)
                            )
                    )
                    // FIX: same typo pattern as the starting pose above.
                    .setLinearHeadingInterpolation(-Math.toRadians(60), -Math.toRadians(84))
                    .build();
        }
    }

    public Command autoRoutine() {
        return sequential(
                follow(follower, paths.MainChain),
                Commands.waitMs(500),

                // FIX: this used to be
                //   Commands.instant(() -> new ParallelCommandGroup(shooter.runAtPowerCommand(0.5), new WaitCommand(500)))
                // which CONSTRUCTS a SolversLib ParallelCommandGroup object and
                // immediately discards it -- nothing ever called .schedule() on it,
                // AND this loop never pumps SolversLib's CommandScheduler in the
                // first place (only Pedro's own Scheduler.execute()), so it could
                // never have run regardless. Replaced with direct method calls
                // sequenced through Pedro's own Commands/Scheduler, which IS being
                // pumped every loop above.
                Commands.instant(() -> shooter.setPower(0.5)),
                Commands.waitMs(500),

                Commands.instant(() -> {
                    shooter.setPower(-0.7);
                    intake.setPower(1.0);
                }),
                Commands.waitMs(3000),

                // FIX (addition): explicit stop step -- nothing previously turned
                // the shooter/intake back off at the end of the routine.
                Commands.instant(() -> {
                    shooter.setPower(0);
                    intake.setPower(0);
                })
        );
    }
}