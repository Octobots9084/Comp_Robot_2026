package frc.robot.commands.auto.StateChange;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Superstructure;

public class SetStateClimbL3 extends InstantCommand {
    public SetStateClimbL3() {
        Superstructure.getInstance().wantedState = States.CLIMB_L3;
    }
}
