package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.StartEndCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.function.DoubleSupplier;

class SimpleMotor extends SubsystemBase {
    private MotorEx m_motor;
    
   

    public SimpleMotor(
        HardwareMap hardwareMap, String name
    ) {
        this.m_motor = new MotorEx(hardwareMap, name);
    }

    public Command setPowerCommand(double power){
        return new StartEndCommand(
                () -> m_motor.set(power),
                () -> m_motor.stopMotor(),
                this
        );
    }
    public Command stopCommand(){
        return new InstantCommand(
                () -> m_motor.stopMotor(),
                this
        );
    }
    

}
