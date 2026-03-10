
package frc.robot.commands.auto.StateChange;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;

public class SetStateManual extends InstantCommand {
    public SetStateManual() {
        // Superstructure.getInstance().wantedState = States.MANUAL;
        Shooter.getInstance().wantedShooterState = ShooterStates.MANUAL;
    }
}
