package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import java.util.HashMap;
import java.util.concurrent.atomic.AtomicInteger;

public class SorterSubsystem extends SubsystemBase {
    HardwareMap m_hardwareMap;
    private SimpleServo m_leverServo;
    private SimpleServo m_sorterServo;
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
    static final double LEVER_RETRACT_ANGLE = 25;
    static final double LEVER_EXTEND_ANGLE = LEVER_RETRACT_ANGLE + 40;
    private int currentCompartment = 0;
    private boolean toIntake = true;
    static Colour[] occupancy = new Colour[]{Colour.NONE, Colour.NONE, Colour.NONE};


    private static final double WAIT_TIME = 200; // in ms
    private NormalizedColorSensor m_colourSensor;

    public SorterSubsystem(HardwareMap hardwareMap) {
        m_leverServo = new SimpleServo(hardwareMap, "leverServo", MIN_ANGLE, MAX_ANGLE);
        m_sorterServo = new SimpleServo(hardwareMap, "sorterServo", MIN_ANGLE, MAX_ANGLE);
        m_colourSensor = hardwareMap.get(NormalizedColorSensor.class, "sorterColour");
        m_hardwareMap = hardwareMap;
    }

    public void setSorterAngle(int position, boolean toIntake){
        setLeverAngle(true);
        currentCompartment = position;
        this.toIntake = toIntake;


        /*
            position:
              0 - Compartment 1
              1 - Compartment 2
              2 - Compartment 3
            toIntake: If true, then turning Compartment towards Intake, otherwise turning towards Lever
         */
        switch (position) {
            case 0:
                m_sorterServo.turnToAngle((toIntake ? C0_INTAKE_ANGLE : C0_LEVER_ANGLE));
                break;
            case 1:
                m_sorterServo.turnToAngle((toIntake ? C1_INTAKE_ANGLE: C1_LEVER_ANGLE));
                break;
            case 2:
                m_sorterServo.turnToAngle((toIntake ? C2_INTAKE_ANGLE : C2_LEVER_ANGLE));
                break;
            default:
                break;
        }
    }

    public void feedUnoccupiedCompartment() {
        for (int i = 0; i < 3; i++) {
            if (occupancy[i] == Colour.NONE) {
                setSorterAngle(i, true);
                return;
            }
        }
    }

    public Command feedUnoccupiedCompartmentCommand() {
        return new InstantCommand(this::feedUnoccupiedCompartment, this)
                .andThen(new WaitCommand((long) WAIT_TIME));
    }

    public void loadIntoShooter(int position) {
        setSorterAngle(position, false);
        occupancy[position] = Colour.NONE;
    }

    public void loadIntoShooter(Colour colour) {
        for (int pos = 0; pos < 3; pos++) {
            if (occupancy[pos] == colour) {
                loadIntoShooter(pos);
                return;
            }
        }
    }

    public Command loadIntoShooterCommand(int position) {
        return new InstantCommand(() -> {
            loadIntoShooterCommand(position);
        }, this)
                .andThen(new WaitCommand((long) WAIT_TIME))
                .andThen(new InstantCommand(() -> { setLeverAngle(false); }, this));
    }


    public Command loadIntoShooterCommand(Colour colour) {
        return new InstantCommand(() -> {
            loadIntoShooterCommand(colour);
        }, this)
                .andThen(new WaitCommand((long) WAIT_TIME))
                .andThen(new InstantCommand(() -> { setLeverAngle(false); }, this));
    }

    public Command getColourCommand(){
        AtomicInteger numPurple = new AtomicInteger();
        AtomicInteger numGreen = new AtomicInteger();
        return new SelectCommand(
                new HashMap<Object, Command>(){{
                    put(0, setAngleCommand(2, false));
                    put(1, setAngleCommand(0, false));
                    put(2, setAngleCommand(1, false));


                }}, () -> {return currentCompartment;}
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
                        ).withTimeout((long) WAIT_TIME)
                )
                .andThen(
                        new InstantCommand(() -> {
                            int readingCompartment =0;
                            switch (currentCompartment){
                                case 0: readingCompartment = 1; break;
                                case 1: readingCompartment = 2; break;
                                case 2: readingCompartment = 0; break;
                                default: break;
                            }
                            if (numGreen.get() > numPurple.get()){
                                occupancy[readingCompartment] = Colour.GREEN;
                            }
                            if (numPurple.get() > numGreen.get()){
                                occupancy[readingCompartment] = Colour.PURPLE;
                            }
                        }


                ));
    }
    public Command setAngleCommand(int position, boolean toIntake){
        return new InstantCommand(() -> {
            setSorterAngle(position, toIntake);
        }, this)
                .andThen(new WaitCommand((long) WAIT_TIME));
    }
    public void setLeverAngle(boolean retract)
    {
        m_leverServo.turnToAngle((retract ? LEVER_RETRACT_ANGLE : LEVER_EXTEND_ANGLE));
    }

    public enum Colour {
        NONE,
        PURPLE,
        GREEN
    }

    private static final double kAlphaThreshold = 0.4;

    private static final double kMinGreenHue = 90;
    private static final double kMaxGreenHue = 180;

    private static final double kMinPurpleHue = 200;
    private static final double kMaxPurpleHue = 330;

    public Colour getColour() {
        NormalizedRGBA colours = m_colourSensor.getNormalizedColors();
        if (colours.alpha < kAlphaThreshold) return Colour.NONE;

        float[] hsvColour = new float[3];
        Color.colorToHSV(colours.toColor(), hsvColour);
        float hue = hsvColour[0];
        if (hue >= kMinGreenHue && hue <= kMaxGreenHue) return Colour.GREEN;
        else if (hue >= kMinPurpleHue && hue <= kMaxPurpleHue) return Colour.PURPLE;
        else return Colour.NONE;
    }
}
