package frc.robot.commands.auto.StateChange;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeStates;

public class SetIntakeStateExtended extends InstantCommand {
    public SetIntakeStateExtended() {
        // Superstructure.getInstance().userRequestedIntakeState = IntakeStates.INTAKING;
        Intake.getInstance().setWantedState(IntakeStates.EXTENDED);
    }
}
