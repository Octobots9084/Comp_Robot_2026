package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter.Shooter;

public class ButtonConfig {
    public static CommandXboxController driverController = new CommandXboxController(0);
    public void initTeleop(){
        driverController.rightTrigger().onTrue(new InstantCommand(
            () -> Shooter.driverOverride = true))
            .onFalse(new InstantCommand(
            () -> Shooter.driverOverride = false
            ));
    }
}
