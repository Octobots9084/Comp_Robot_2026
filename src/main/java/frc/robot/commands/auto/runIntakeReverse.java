package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeStates;

public class runIntakeReverse extends Command {
    Intake intake;

    @Override
    public void initialize() {
        intake = Intake.getInstance();
    }

    @Override
    public void execute() {
        intake.setWantedState(IntakeStates.REVERSEINTAKING);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setWantedState(IntakeStates.EXTENDED);
    }
}
