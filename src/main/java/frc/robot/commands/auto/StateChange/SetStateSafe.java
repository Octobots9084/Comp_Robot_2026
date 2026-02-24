package frc.robot.commands.auto.StateChange;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Superstructure;

public class SetStateSafe extends InstantCommand {
    public SetStateSafe() {
        Superstructure.getInstance().wantedState = States.SAFE;
    }
}
