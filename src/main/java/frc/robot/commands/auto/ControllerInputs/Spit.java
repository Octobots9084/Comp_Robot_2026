package frc.robot.commands.auto.ControllerInputs;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;

public class Spit extends InstantCommand{
    public Spit () {
        if (Superstructure.getInstance().currentState == States.MANUAL) {
            Shooter.driverOverride = false; //TODO update to true
        }else if(Superstructure.getInstance().currentState != States.MANUAL){
            Shooter.driverOverride = false;
        }else{
            Shooter.getInstance().wantedShooterState = ShooterStates.SPIT;
        }
    }
}
