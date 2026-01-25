package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ShooterConstants;

public class ButtonConfig {
    CommandXboxController driverController = new CommandXboxController(0);
    public void initTeleop(){
        driverController.rightTrigger().onTrue(new InstantCommand(
            () -> ShooterConstants.driverShoot = true))
            .onFalse(new InstantCommand(
            () -> ShooterConstants.driverShoot = false
            ));
    }
}
