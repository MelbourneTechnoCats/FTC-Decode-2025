package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@TeleOp
public class VisionOpMode extends CommandOpMode {
    private VisionSubsystem m_visionSubsystem;

    @Override
    public void initialize() {
        m_visionSubsystem = new VisionSubsystem(hardwareMap, telemetry);
    }
}
