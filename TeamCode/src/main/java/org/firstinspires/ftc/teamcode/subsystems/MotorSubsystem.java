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

public class MotorSubsystem extends SubsystemBase {
    private MotorEx m_motor;
    private PIDController m_pidController; // PID (closed-loop) controller
    private SimpleMotorFeedforward m_ffController; // feedforward (open-loop) controller

    private static final double kDefaultKP = 0.0;
    private static final double kDefaultKI = 0.0;
    private static final double kDefaultKD = 0.0;
    private static final double kDefaultKS = 0.0;
    private static final double kDefaultKV = 0.0;
    private static final double kDefaultKA = 0.0;

    private static final double kDefaultVelTolerance = 0.05;

    private double m_velTolerance;


    public MotorSubsystem(
            HardwareMap hardwareMap, String name, double encoderResolution /* in cpr */
    ) {
        this(
                hardwareMap, name, encoderResolution, false,
                kDefaultKP, kDefaultKI, kDefaultKD, kDefaultKS, kDefaultKV, kDefaultKA,
                kDefaultVelTolerance
        );
    }

    
    public MotorSubsystem(
            HardwareMap hardwareMap, String name, double encoderResolution, /* in cpr */ boolean inverted
    ) {
        this(
                hardwareMap, name, encoderResolution, inverted,
                kDefaultKP, kDefaultKI, kDefaultKD, kDefaultKS, kDefaultKV, kDefaultKA,
                kDefaultVelTolerance
        );
    }

    
    public MotorSubsystem(
            HardwareMap hardwareMap, String name, double encoderResolution, /* in cpr */
            double velTolerance
    ) {
        this(
                hardwareMap, name, encoderResolution, false,
                kDefaultKP, kDefaultKI, kDefaultKD, kDefaultKS, kDefaultKV, kDefaultKA,
                velTolerance
        );
    }

    
    public MotorSubsystem(
            HardwareMap hardwareMap, String name, double encoderResolution, /* in cpr */ boolean inverted,
            double velTolerance
    ) {
        this(
                hardwareMap, name, encoderResolution, inverted,
                kDefaultKP, kDefaultKI, kDefaultKD, kDefaultKS, kDefaultKV, kDefaultKA,
                velTolerance
        );
    }

   
    public MotorSubsystem(
            HardwareMap hardwareMap, String name, double encoderResolution, /* in cpr */
            double kP, double kI, double kD, double kS, double kV, double kA
    ) {
        this(
                hardwareMap, name, encoderResolution, false,
                kP, kI, kD, kS, kV, kA,
                kDefaultVelTolerance
        );

    }

    
    public MotorSubsystem(
            HardwareMap hardwareMap, String name, double encoderResolution, /* in cpr */ boolean inverted,
            double kP, double kI, double kD, double kS, double kV, double kA
    ) {
        this(
                hardwareMap, name, encoderResolution, inverted,
                kP, kI, kD, kS, kV, kA,
                kDefaultVelTolerance
        );

    }

    HardwareMap.DeviceMapping<VoltageSensor> m_voltageSensors;

    private final double m_encoderResolution;

    
    public MotorSubsystem(
            HardwareMap hardwareMap, String name, double encoderResolution, /* in cpr */ boolean inverted,
            double kP, double kI, double kD, double kS, double kV, double kA,
            double velTolerance
    ) {
        // max RPM argument is not used because we implement our own velocity controller
        m_motor = new MotorEx(hardwareMap, name, encoderResolution, 0);
        m_motor.setCachingTolerance(0.05);
        m_motor.setInverted(inverted);
        m_encoderResolution = encoderResolution;

        m_pidController = new PIDController(kP, kI, kD);
        m_ffController = new SimpleMotorFeedforward(kS, kV, kA);

        m_voltageSensors = hardwareMap.voltageSensor;
        m_velTolerance = velTolerance;
    }

   
    public void setPIDCoefficients(double kP, double kI, double kD) {
        m_pidController.setPID(kP, kI, kD);
    }

    
    public void setFFCoefficients(double kS, double kV, double kA) {
        m_ffController = new SimpleMotorFeedforward(kS, kV, kA);

    }

    private double m_targetVelocity = Double.NaN;

    
    public double getTargetVelocity() {
        return m_targetVelocity;
    }

   
    public Command setPowerCommand(double power) {
        return new StartEndCommand(
                () -> { m_motor.set(power); m_targetVelocity = Double.NaN; },
                () -> { m_motor.set(0); },
                this
        );
    }

    
    public Command setPowerCommand(DoubleSupplier power) { // will never stop by itself unless interrupted
        return new FunctionalCommand(
                () -> { m_targetVelocity = Double.NaN; },
                () -> { m_motor.set(power.getAsDouble()); },
                (Boolean interrupted) -> { m_motor.set(0); },
                () -> false,
                this
        );
    }

    
    public double getVelocity() {
        return m_motor.getCorrectedVelocity()  * 60 / m_encoderResolution;
    }

    
     private double getBatteryVoltage() {
         double result = Double.POSITIVE_INFINITY;
         for (VoltageSensor sensor : m_voltageSensors) {
             double voltage = sensor.getVoltage();
             if (voltage > 0) {
                 result = Math.min(result, voltage);
             }
         }
         return result;
     }

    
     public Command setVelocityCommand(double velocity) {
         return new FunctionalCommand(
                 () -> { m_targetVelocity = velocity; },
                 () -> {
                     double voltage = getBatteryVoltage();
                     double power = (m_pidController.calculate(getVelocity(), velocity) + m_ffController.calculate(velocity)) / voltage;
                     m_motor.set(Math.max(-1, Math.min(1, power)));
                 },
                 (Boolean interrupted) -> { m_motor.set(0); m_targetVelocity = Double.NaN; },
                 () -> false,
                 this
         );
     }

    
    public void setInverted(boolean inverted) {
        m_motor.setInverted(inverted);
    }


    public void setRawPower(double power) {
        m_targetVelocity = Double.NaN;
        m_motor.set(power);
    }

    
    public double getVelocityTolerance() {
        return m_velTolerance;
    }

    
    public void setVelocityTolerance(double value) {
        m_velTolerance = value;
    }
    public double getPower(){return m_motor.get();}

    
    public boolean isVelocityReached() {
        if (Double.isNaN(m_targetVelocity)) return true; // not being controlled by velocity
        return Math.abs((m_targetVelocity - getVelocity()) / m_targetVelocity) <= m_velTolerance;
    }
}
