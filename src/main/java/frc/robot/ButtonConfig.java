package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ButtonConfig {
    CommandXboxController driverController = new CommandXboxController(0);
    public void initTeleop(){
        driverController.rightTrigger().onTrue(new InstantCommand(
            () -> Constants.driverShoot = true))
            .onFalse(new InstantCommand(
            () -> Constants.driverShoot = false
            ));
    }
}
