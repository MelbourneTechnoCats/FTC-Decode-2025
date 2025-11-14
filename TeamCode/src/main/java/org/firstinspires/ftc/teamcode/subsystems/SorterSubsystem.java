package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;

import org.firstinspires.ftc.teamcode.ArrayIndexComparator;
import org.firstinspires.ftc.teamcode.commands.SequentialCommandGroup; // patched SequentialCommandGroup
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import java.util.Arrays;
import java.util.HashMap;
import java.util.concurrent.atomic.AtomicInteger;

public class SorterSubsystem extends SubsystemBase {
    HardwareMap m_hardwareMap;
    private ServoSubsystem m_leverServo;
    private ServoSubsystem m_sorterServo;
    static final double MIN_ANGLE = 0;
    static final double MAX_ANGLE = 300;
//    static final double C2_LEVER_ANGLE = 180;
//    static final double C0_INTAKE_ANGLE = C2_LEVER_ANGLE + 60;
//    static final double C1_INTAKE_ANGLE = C2_LEVER_ANGLE - 60;
//    static final double C2_LEVER_ANGLE = C2_LEVER_ANGLE - 180;
//    static final double C0_LEVER_ANGLE = C2_LEVER_ANGLE - 120;
//    static final double C1_LEVER_ANGLE = C2_LEVER_ANGLE + 120;
    static final double C2_LEVER_ANGLE = 180;
    static final double C0_INTAKE_ANGLE = C2_LEVER_ANGLE + 60;
    static final double C1_INTAKE_ANGLE = C2_LEVER_ANGLE - 60;
    static final double C2_INTAKE_ANGLE = C2_LEVER_ANGLE - 180;
    static final double C0_LEVER_ANGLE = C2_LEVER_ANGLE - 120;
    static final double C1_LEVER_ANGLE = C2_LEVER_ANGLE + 120;
    static final double LEVER_RETRACT_ANGLE = 40;
    static final double LEVER_EXTEND_ANGLE = LEVER_RETRACT_ANGLE + 80;
    private int currentCompartment = 0;
    private boolean toIntake = true;
    public Colour[] occupancy = new Colour[]{Colour.NONE, Colour.NONE, Colour.NONE};

    static final double SENSOR_WAIT_TIME = 200; // in ms
    private NormalizedColorSensor m_colourSensor;

    private static float COLOUR_SENSOR_GAIN = 21;

    public static double LEVER_SERVO_SPEED = 100; // GoBilda Dual Mode Speed servo no-load speed @ 6.0V
    public static double SORTER_SERVO_SPEED = 40; // GoBilda Dual Mode Torque servo no-load speed @ 6.0V
    // NOTE: both servos are now powered by the REV Servo Hub

    static final Double[] INTAKE_ANGLES = new Double[]{C0_INTAKE_ANGLE, C1_INTAKE_ANGLE, C2_INTAKE_ANGLE};
    static final Integer[] INTAKE_ANGLE_ORDER = new ArrayIndexComparator<>(INTAKE_ANGLES).getSortedIndices();

    static final Double[] LEVER_ANGLES = new Double[]{C0_LEVER_ANGLE, C1_LEVER_ANGLE, C2_LEVER_ANGLE};
    static final Integer[] LEVER_ANGLE_ORDER = new ArrayIndexComparator<>(LEVER_ANGLES).getSortedIndices();

    static final Double[] READ_COLOUR_ANGLES = new Double[]{C2_LEVER_ANGLE, C0_LEVER_ANGLE, C1_LEVER_ANGLE};
    static final Integer[] READ_COLOUR_ANGLE_ORDER = new ArrayIndexComparator<>(LEVER_ANGLES).getSortedIndices();

    public SorterSubsystem(HardwareMap hardwareMap) {
        m_leverServo = new ServoSubsystem(hardwareMap, "leverServo", LEVER_SERVO_SPEED, MIN_ANGLE, MAX_ANGLE);
        m_sorterServo = new ServoSubsystem(hardwareMap, "sorterServo", SORTER_SERVO_SPEED, MIN_ANGLE, MAX_ANGLE);
        m_colourSensor = hardwareMap.get(NormalizedColorSensor.class, "sorterColour");
        m_colourSensor.setGain(COLOUR_SENSOR_GAIN);
        m_hardwareMap = hardwareMap;

        setDefaultCommand(new RunCommand(() -> {
            if (!toIntake) {
                /* not facing intake - a compartment is facing the colour sensor, so we can read it now */
                int readingCompartment = 0;
                switch (currentCompartment) {
                    case 0:
                        readingCompartment = 1;
                        break;
                    case 1:
                        readingCompartment = 2;
                        break;
                    case 2:
                        readingCompartment = 0;
                        break;
                    default:
                        break;
                }
                occupancy[readingCompartment] = getColour();
            }
        }, this));
    }

    public Command getColourCommand(int compartment) {
        AtomicInteger numPurple = new AtomicInteger();
        AtomicInteger numGreen = new AtomicInteger();
        return new SelectCommand(
                new HashMap<Object, Command>(){{
                    put(0, setSorterAngleCommand(2, false));
                    put(1, setSorterAngleCommand(0, false));
                    put(2, setSorterAngleCommand(1, false));


                }}, () -> compartment
        )
                .andThen(new InstantCommand(() -> {
                    numPurple.set(0);
                    numGreen.set(0);
                }))
                .andThen(
                        new RunCommand(
                                () -> {
                                    Colour colour = getColour();
                                    switch (colour) {
                                        case PURPLE:
                                            numPurple.incrementAndGet();
                                            break;
                                        case GREEN:
                                            numGreen.incrementAndGet();
                                            break;
                                        default:
                                            break;
                                    }
                                }
                        ).withTimeout((long) SENSOR_WAIT_TIME)
                )
                .andThen(
                        new InstantCommand(() -> {
                            if (numGreen.get() > numPurple.get()){
                                occupancy[compartment] = Colour.GREEN;
                            }
                            else if (numPurple.get() > numGreen.get()){
                                occupancy[compartment] = Colour.PURPLE;
                            }
                            else occupancy[compartment] = Colour.NONE;
                        }


                ));
    }
    public Command setSorterAngleCommand(int position, boolean toIntake){
        return new SequentialCommandGroup(
                setLeverAngleCommand(true), // TODO: maybe we want to start moving before it pops all the way down?
                new SelectCommand(() -> {
                    /*
                        position:
                          0 - Compartment 1
                          1 - Compartment 2
                          2 - Compartment 3
                        toIntake: If true, then turning Compartment towards Intake, otherwise turning towards Lever
                     */
                    switch (position) {
                        case 0:
                            return m_sorterServo.setAngleCommand((toIntake ? C0_INTAKE_ANGLE : C0_LEVER_ANGLE));
                        case 1:
                            return m_sorterServo.setAngleCommand((toIntake ? C1_INTAKE_ANGLE: C1_LEVER_ANGLE));
                        case 2:
                            return m_sorterServo.setAngleCommand((toIntake ? C2_INTAKE_ANGLE : C2_LEVER_ANGLE));
                        default:
                            return new InstantCommand(() -> {}); // no-op
                    }
                }),
                new InstantCommand(() -> {
                    currentCompartment = position;
                    this.toIntake = toIntake;
                })
        );
    }

    public Command setLeverAngleCommand(boolean retract)
    {
        return m_leverServo.setAngleCommand((retract ? LEVER_RETRACT_ANGLE : LEVER_EXTEND_ANGLE));
    }

    public enum Colour {
        NONE,
        PURPLE,
        GREEN
    }

    private static final double kAlphaMinThreshold = 0.1;
    private static final double kAlphaMaxThreshold = 0.16;

    private static final double kMinGreenHue = 90;
    private static final double kMaxGreenHue = 180;

    private static final double kMinPurpleHue = 200;
    private static final double kMaxPurpleHue = 330;

    public Colour getColour() {
        NormalizedRGBA colours = m_colourSensor.getNormalizedColors();
        if (colours.alpha >= kAlphaMinThreshold && colours.alpha <= kAlphaMaxThreshold) return Colour.NONE;

        float[] hsvColour = new float[3];
        Color.colorToHSV(colours.toColor(), hsvColour);
        float hue = hsvColour[0];
        if (hue >= kMinGreenHue && hue <= kMaxGreenHue) return Colour.GREEN;
        else if (hue >= kMinPurpleHue && hue <= kMaxPurpleHue) return Colour.PURPLE;
        else return Colour.NONE;
    }

    public int getCurrentCompartment() {
        return currentCompartment;
    }

    public boolean getIntakePosition() {
        return toIntake;
    }

    public double getSorterServoPosition() {
        return m_sorterServo.getCurrentPosition();
    }
}
