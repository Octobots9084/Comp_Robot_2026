package frc.robot.commands.auto.StateChange;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeStates;

public class SetIntakeStateReverse extends InstantCommand {
    public SetIntakeStateReverse() {
        Intake.getInstance().wantedState = IntakeStates.REVERSEINTAKING;
    }
}
