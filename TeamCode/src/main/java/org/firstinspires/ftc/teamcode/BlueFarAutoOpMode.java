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
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.Timer;

@Autonomous(name = "blue far auto - wait 20s then leave", group = "Autonomous")
@Configurable // Panels
public class BlueFarAutoOpMode extends LinearOpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private LimelightSubsystem limelight;
    private TurretSubsystem turret;
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
        turret = new TurretSubsystem(hardwareMap, telemetry);


        follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));
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
//        schedule(new AutoCommands(hardwareMap,telemetry).shootSequence());
        schedule(autoRoutine());
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
        public PathChain leave;

        public Paths(Follower follower) {
            leave = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(0.000, 0.000),
                                    new Pose(0.000, 26.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
        }
    }

    public Command autoRoutine() {
        return sequential(
                Commands.waitMs(5000),
                follow(follower, paths.leave)



        );
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;

    }
}
