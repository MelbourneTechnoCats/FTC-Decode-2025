package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class SorterSubsystem extends SubsystemBase {
    HardwareMap m_hardwareMap;
    private SimpleServo m_leverServo;
    private SimpleServo m_sorterServo;
    static final double MIN_ANGLE = 0;
    static final double MAX_ANGLE = 300;
//    static final double C2_LEVER_ANGLE = 180;
//    static final double C0_INTAKE_ANGLE = C2_LEVER_ANGLE + 60;
//    static final double C1_INTAKE_ANGLE = C2_LEVER_ANGLE - 60;
//    static final double C2_INTAKE_ANGLE = C2_LEVER_ANGLE - 180;
//    static final double C0_LEVER_ANGLE = C2_LEVER_ANGLE - 120;
//    static final double C1_LEVER_ANGLE = C2_LEVER_ANGLE + 120;
    static final double C2_INTAKE_ANGLE = 180;
    static final double C0_INTAKE_ANGLE = C2_INTAKE_ANGLE + 60;
    static final double C1_INTAKE_ANGLE = C2_INTAKE_ANGLE - 60;
    static final double C2_LEVER_ANGLE = C2_INTAKE_ANGLE - 180;
    static final double C0_LEVER_ANGLE = C2_INTAKE_ANGLE - 120;
    static final double C1_LEVER_ANGLE = C2_INTAKE_ANGLE + 120;
    static final double LEVER_RETRACT_ANGLE = 25;
    static final double LEVER_EXTEND_ANGLE = LEVER_RETRACT_ANGLE + 40;

    private NormalizedColorSensor m_colourSensor;

    public SorterSubsystem(HardwareMap hardwareMap) {
        m_leverServo = new SimpleServo(hardwareMap, "leverServo", MIN_ANGLE, MAX_ANGLE);
        m_sorterServo = new SimpleServo(hardwareMap, "sorterServo", MIN_ANGLE, MAX_ANGLE);
        m_colourSensor = hardwareMap.get(NormalizedColorSensor.class, "sorterColour");
        m_hardwareMap = hardwareMap;
    }

    public void setSorterAngle(int position, boolean toIntake){
        setLeverAngle(true);

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
