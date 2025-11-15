package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.util.Timing;

import java.util.concurrent.TimeUnit;

public class ActionCommand extends CommandBase {
    private final Action action;
    private boolean finished = false;
    public ActionCommand(Action action, Subsystem... requirements) {
        this.action = action;
        addRequirements(requirements);
    }

    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        action.preview(packet.fieldOverlay());
        finished = !action.run(packet);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        Timing.Timer timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);
        timer.start();
        while (!timer.done());
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}