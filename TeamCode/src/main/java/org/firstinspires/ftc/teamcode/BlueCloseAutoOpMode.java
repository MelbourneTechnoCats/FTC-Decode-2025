package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;

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

import java.util.Timer;

@Autonomous(name = "blue close auto", group = "Autonomous")
@Configurable // Panels
public class BlueCloseAutoOpMode extends LinearOpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private LimelightSubsystem limelight;
    private HoodSubsystem hood;
    private int pathState; // Current autonomous path state (state machine)
    private Timer pathTimer, opTimer;
    private Paths paths; // Paths defined in the Paths class



    @Override
    public void runOpMode() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        shooter = new ShooterSubsystem(hardwareMap, telemetry, intake, hood, limelight);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));
        pathTimer = new Timer();
        opTimer = new Timer();

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        //These will run when the OpMode is initiated
        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);



        waitForStart();
        //We schedule all our commands when we start the OpMode
        schedule(new AutoCommands(hardwareMap,telemetry).shootWithIntakePulses());
        while (opModeIsActive()) {
            //Update the follower and execute the scheduler every loop
            follower.update();
            Scheduler.execute();

            // Feedback to Driver Hub for debugging
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }

    public static class Paths {
        public PathChain auto;
        public PathChain after;

        public Paths(Follower follower) {
            auto = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(41.538, 140.851),
                                    new Pose(78.402, 104.198)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(78.402, 104.198),
                                    new Pose(85.632, 108.428)
                            )
                    )
                    .setTangentHeadingInterpolation()

                    .build();
            after = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(85.632, 108.428),
                                    new Pose(60.158, 37.311)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

        }
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
                Commands.waitMs(500),
                Commands.instant(() -> { shooter.setPower(-1); intake.setPower(0.0); }),
                Commands.waitMs(500),

                // pulse 2
                Commands.instant(() -> { shooter.setPower(-1); intake.setPower(1.0); }),
                Commands.waitMs(500),
                Commands.instant(() -> { shooter.setPower(-1); intake.setPower(0.0); }),
                Commands.waitMs(500),

                // pulse 3
                Commands.instant(() -> { shooter.setPower(-1); intake.setPower(1.0); }),
                Commands.waitMs(500),
                Commands.instant(() -> { shooter.setPower(-1); intake.setPower(0.0); }),
                Commands.waitMs(500),

                // stop everything at the end
                Commands.instant(() -> {
                    shooter.setPower(0);
                    intake.setPower(0);
                })
        );
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;

    }
}