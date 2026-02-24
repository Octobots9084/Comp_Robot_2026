package frc.robot.commands.auto.StateChange;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Superstructure;

public class CommandStateSafe extends InstantCommand {
    public CommandStateSafe() {
        Superstructure.getInstance().wantedState = States.SAFE;
    }
}
