package frc.robot.commands.auto.StateChange;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeStates;

public class SetIntakeStateIntaking extends InstantCommand {
    public SetIntakeStateIntaking() {
        Intake.getInstance().wantedState = IntakeStates.INTAKING;
    }
}
