package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.commands.SequentialCommandGroup;

import java.util.ArrayList;
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

    private int getClosestCompartment(ArrayList<Integer> compartments) {
        final double currentPosition = m_sorter.getSorterServoPosition();
        int compartment = -1;
        if (Double.isNaN(currentPosition)) compartment = compartments.get(0); // get any compartment since we don't know the current servo pos yet
        else {
            /* find closest compartment */
            double minPositionDelta = Double.POSITIVE_INFINITY;
            for (Integer iterCompartment : compartments) {
                double delta = Math.abs(currentPosition - SorterSubsystem.INTAKE_ANGLES[iterCompartment]);
                if (delta < minPositionDelta) {
                    minPositionDelta = delta;
                    compartment = iterCompartment;
                }
            }
        }
        return compartment;
    }

    public Command intakeCommand() {
        return new SelectCommand(() -> {
            /* get empty compartments */
            ArrayList<Integer> empty = new ArrayList<>(3); // empty compartments' indices
            for (int i = 0; i < 3; i++) {
                if (m_sorter.occupancy[i] == SorterSubsystem.Colour.NONE)
                    empty.add(i);
            }

            int compartment = getClosestCompartment(empty);
            if (compartment < 0) return new InstantCommand(() -> {}); // no empty compartments - no-op

            return setSorterAngleCommand(compartment, true) // find unoccupied sorter compartment
                    .andThen(m_intake.runCommand().interruptOn(m_intake::isBallThere)) // load ball in
                    .andThen(getColourCommand(compartment)); // finally update occupancy
        });
    }

    public Command loadIntoShooterCommand(int position) { // by position
        return setSorterAngleCommand(position, false)
                .andThen(
                        m_sorter.setLeverAngleCommand(false)
                                .alongWith(new InstantCommand(() -> {
                                    m_sorter.occupancy[position] = SorterSubsystem.Colour.NONE;
                                }))
                )
                .andThen(m_sorter.setLeverAngleCommand(true));
    }

    public Command loadIntoShooterCommand(SorterSubsystem.Colour colour) { // by closest compartment containing colour
        return new SelectCommand(() -> {
            /* get suitable compartments */
            ArrayList<Integer> compartments = new ArrayList<>(3); // suitable compartments' indices
            for (int i = 0; i < 3; i++) {
                if (m_sorter.occupancy[i] == colour)
                    compartments.add(i);
            }

            int compartment = getClosestCompartment(compartments);
            if (compartment == -1) return new InstantCommand(() -> {});
            return loadIntoShooterCommand(compartment);
        });
    }

    public Command loadIntoShooterCommand(SorterSubsystem.Colour colour, boolean strict) {
        if (strict || m_sorter.occupancy[0] == colour || m_sorter.occupancy[1] == colour || m_sorter.occupancy[2] == colour) {
            return loadIntoShooterCommand(colour);
        }
        else{
            return loadIntoShooterCommand();
        }
    }

    public Command loadIntoShooterCommand() { // by closest compartment containing any colour
        return new SelectCommand(() -> {
            /* get suitable compartments */
            ArrayList<Integer> compartments = new ArrayList<>(3); // suitable compartments' indices
            for (int i = 0; i < 3; i++) {
                if (m_sorter.occupancy[i] != SorterSubsystem.Colour.NONE)
                    compartments.add(i);
            }

            int compartment = getClosestCompartment(compartments);
            if (compartment == -1) return new InstantCommand(() -> {});
            return loadIntoShooterCommand(compartment);
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
