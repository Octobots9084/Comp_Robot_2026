package frc.robot.commands.auto.StateChange;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Superstructure;

public class SetStateClimb extends InstantCommand{
    public SetStateClimb () {
        Superstructure.getInstance().wantedState = States.CLIMB;
    }
}
