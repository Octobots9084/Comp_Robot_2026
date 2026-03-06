package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.runIntake;
import frc.robot.commands.auto.ControllerInputs.Spit;
import frc.robot.commands.auto.StateChange.*;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Shooter.Shooter;

public class ButtonConfig {
    public static CommandXboxController driverController = new CommandXboxController(0);
    public static CommandXboxController coDriverController = new CommandXboxController(1);
    public Superstructure superstructure = Superstructure.getInstance();

    public void initTeleop() {

        
        // driverController.leftTrigger().onTrue(new SetIntakeStateIntaking()).onFalse(new SetIntakeStateSafe());

        driverController.leftBumper().whileTrue(new SetIntakeStateReverse()).toggleOnFalse(new SetIntakeStateExtended());

        driverController.leftTrigger().whileTrue(new runIntake());

        driverController.y().onTrue(new InstantCommand(() ->
        {SwerveSubsystem.getInstance().io.zeroGyro();}));

        // Climb currently not implemented
        // driverController.b().onTrue(new SetStateClimbL3()); //TODO - implement climb functions
        // driverController.a().onTrue(new SetStateUnclimb());

        driverController.rightTrigger(0.5).onTrue(new SetStateShooter());
        driverController.rightTrigger(0.5).onTrue(new InstantCommand(
                () -> Shooter.driverOverride = true))
                .onFalse(new InstantCommand(
                        () -> Shooter.driverOverride = false));

        coDriverController.b().onTrue(new SetStateManual());
        // coDriverController.leftTrigger(0.5).onTrue(new SetStateSafe());
        coDriverController.rightBumper().onTrue(new Spit());
        coDriverController.rightTrigger(0.5).onTrue(new SetStateShooter());
        coDriverController.rightTrigger().onTrue(new InstantCommand(
                () -> Shooter.driverOverride = true))
                .onFalse(new InstantCommand(
                        () -> Shooter.driverOverride = false));

    }
}
