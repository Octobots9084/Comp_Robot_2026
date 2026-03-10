package frc.robot.commands.auto.StateChange;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Superstructure;

public class SetStateShooter extends InstantCommand {
    public SetStateShooter() {
        Superstructure.getInstance().wantedState = States.SHOOTER;
    }
}
