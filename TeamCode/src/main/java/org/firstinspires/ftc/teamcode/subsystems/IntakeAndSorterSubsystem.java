package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

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
        return new InstantCommand(m_intake::runMotor, m_intake)
                .andThen(m_sorter.setSorterAngleCommand(position, toIntake))
                .whenFinished(m_intake::stopMotor);
    }

    public Command getColourCommand(){ // uses the above setSorterAngleCommand
        AtomicInteger numPurple = new AtomicInteger();
        AtomicInteger numGreen = new AtomicInteger();
        return new SelectCommand(
                new HashMap<Object, Command>(){{
                    put(0, setSorterAngleCommand(2, false));
                    put(1, setSorterAngleCommand(0, false));
                    put(2, setSorterAngleCommand(1, false));
                }},
                () -> m_sorter.currentCompartment
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
                        ).withTimeout((long) m_sorter.SENSOR_WAIT_TIME)
                )
                .andThen(
                        new InstantCommand(() -> {
                            int readingCompartment = 0;
                            switch (m_sorter.currentCompartment) {
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
                            if (numGreen.get() > numPurple.get()) {
                                m_sorter.occupancy[readingCompartment] = SorterSubsystem.Colour.GREEN;
                            }
                            if (numPurple.get() > numGreen.get()) {
                                m_sorter.occupancy[readingCompartment] = SorterSubsystem.Colour.PURPLE;
                            }
                        }, m_sorter)
                );
    }

    public Command intakeCommand() {
        return new SelectCommand(() -> {
            for (int i = 0; i < 3; i++) {
                if (m_sorter.occupancy[i] == SorterSubsystem.Colour.NONE) {
                    return setSorterAngleCommand(i, true) // find unoccupied sorter compartment
                            .andThen(m_intake.runCommand().interruptOn(m_intake::isBallThere)) // load ball in
                            .andThen(getColourCommand()); // finally update occupancy
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
}
