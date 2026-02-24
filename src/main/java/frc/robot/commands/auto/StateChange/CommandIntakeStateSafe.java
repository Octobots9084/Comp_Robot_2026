package frc.robot.commands.auto.StateChange;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeStates;

public class CommandIntakeStateSafe extends InstantCommand {
    public CommandIntakeStateSafe() {
        Intake.getInstance().wantedState = IntakeStates.SAFE;
    }
}
