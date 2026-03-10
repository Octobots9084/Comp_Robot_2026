package frc.robot.commands.auto.StateChange;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Superstructure;

public class SetStateUnclimb extends InstantCommand {
    public SetStateUnclimb() {
        Superstructure.getInstance().wantedState = States.UNCLIMB;
    }
}
