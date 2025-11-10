package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.commands.SequentialCommandGroup;

import java.util.HashMap;
import java.util.concurrent.atomic.AtomicInteger;

public class IntakeAndSorterSubsystem extends SubsystemBase {
    private IntakeSubsystem m_intake;
    private SorterSubsystem m_sorter;

    public IntakeAndSorterSubsystem(IntakeSubsystem intake, SorterSubsystem sorter) {
        m_intake = intake;
        m_sorter = sorter;
    }

    public Command setSorterAngleCommand(int position, boolean toIntake) { // this wrapper turns the intake to hold the balls in place while turning the sorter
        return m_intake.runCommand()
                .raceWith(m_sorter.setSorterAngleCommand(position, toIntake));
    }

    public Command getColourCommand(int compartment) { // uses the above setSorterAngleCommand
        AtomicInteger numPurple = new AtomicInteger();
        AtomicInteger numGreen = new AtomicInteger();
        return new SelectCommand(
                new HashMap<Object, Command>(){{
                    put(0, setSorterAngleCommand(2, false));
                    put(1, setSorterAngleCommand(0, false));
                    put(2, setSorterAngleCommand(1, false));
                }},
                () -> compartment
        )
                .andThen(new InstantCommand(() -> {
                    numPurple.set(0);
                    numGreen.set(0);
                }))
                .andThen(
                        new RunCommand(
                                () -> {
                                    SorterSubsystem.Colour colour = m_sorter.getColour();
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
                                }, m_sorter
                        ).withTimeout((long) SorterSubsystem.SENSOR_WAIT_TIME)
                )
                .andThen(
                        new InstantCommand(() -> {
                            if (numGreen.get() > numPurple.get()) {
                                m_sorter.occupancy[compartment] = SorterSubsystem.Colour.GREEN;
                            }
                            else if (numPurple.get() > numGreen.get()) {
                                m_sorter.occupancy[compartment] = SorterSubsystem.Colour.PURPLE;
                            }
                            else m_sorter.occupancy[compartment] = SorterSubsystem.Colour.NONE;
                        }, m_sorter)
                );
    }

    public Command intakeCommand() {
        return new SelectCommand(() -> {
            for (int i = 0; i < 3; i++) {
                if (m_sorter.occupancy[i] == SorterSubsystem.Colour.NONE) {
                    int compartment = i; // to keep Java happy
                    return setSorterAngleCommand(compartment, true) // find unoccupied sorter compartment
                            .andThen(m_intake.runCommand().interruptOn(m_intake::isBallThere)) // load ball in
                            .andThen(new SelectCommand(() -> getColourCommand(compartment))); // finally update occupancy
                }
            }
            return new InstantCommand(() -> {}); // no-op
        });
    }

    public Command loadIntoShooterCommand(int position) {
        return setSorterAngleCommand(position, false)
                .andThen(
                        m_sorter.setLeverAngleCommand(false)
                                .alongWith(new InstantCommand(() -> {
                                    m_sorter.occupancy[position] = SorterSubsystem.Colour.NONE;
                                }))
                )
                .andThen(m_sorter.setLeverAngleCommand(true));
    }

    public Command loadIntoShooterCommand(SorterSubsystem.Colour colour) {
        return new SelectCommand(() -> {
            for (int pos = 0; pos < 3; pos++) {
                if (m_sorter.occupancy[pos] == colour) {
                    return loadIntoShooterCommand(pos);
                }
            }
            return new InstantCommand(() -> {}); // no-op
        });
    }

    public Command getAllColoursCommand() {
        return new SelectCommand(() -> {
            if (m_sorter.getSorterServoPosition() > SorterSubsystem.READ_COLOUR_ANGLES[SorterSubsystem.READ_COLOUR_ANGLE_ORDER[1]])
                return new SequentialCommandGroup(
                        getColourCommand(SorterSubsystem.READ_COLOUR_ANGLE_ORDER[2]),
                        getColourCommand(SorterSubsystem.READ_COLOUR_ANGLE_ORDER[1]),
                        getColourCommand(SorterSubsystem.READ_COLOUR_ANGLE_ORDER[0])
                ); // past middle angle - go to max then iterate back to min position
            else
                return new SequentialCommandGroup(
                        getColourCommand(SorterSubsystem.READ_COLOUR_ANGLE_ORDER[0]),
                        getColourCommand(SorterSubsystem.READ_COLOUR_ANGLE_ORDER[1]),
                        getColourCommand(SorterSubsystem.READ_COLOUR_ANGLE_ORDER[2])
                );
        });
    }
}
