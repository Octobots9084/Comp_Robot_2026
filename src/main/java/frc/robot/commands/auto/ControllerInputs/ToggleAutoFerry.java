package frc.robot.commands.auto.ControllerInputs;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;

public class ToggleAutoFerry extends InstantCommand{
    @Override
    public void initialize(){
        if (Shooter.ferryOverride){
            Shooter.ferryOverride = false;
            Shooter.flywheelDebouncer = Shooter.flywheelToleranceThreshold;
        } else {
            Shooter.ferryOverride = true;
            Superstructure.getInstance().wantedState = States.SHOOTER;
            Shooter.getInstance().wantedShooterState = ShooterStates.FERRY;
        }
    }
}
