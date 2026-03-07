package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.runIntake;
import frc.robot.commands.auto.runIntakeReverse;
import frc.robot.commands.auto.ControllerInputs.Spit;
import frc.robot.commands.auto.StateChange.*;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;

public class ButtonConfig {
    public static CommandXboxController driverController = new CommandXboxController(0);
    public static CommandXboxController coDriverController = new CommandXboxController(1);
    public Superstructure superstructure = Superstructure.getInstance();

    public void initTeleop() {

        
        // driverController.leftTrigger().onTrue(new SetIntakeStateIntaking()).onFalse(new SetIntakeStateSafe());

        driverController.leftBumper().whileTrue(new runIntakeReverse());
        driverController.leftTrigger().whileTrue(new runIntake());


        driverController.rightBumper().onTrue(new InstantCommand(() -> {
                superstructure.wantedState = States.UNJAM;
        })).onFalse(new InstantCommand(() -> {
                superstructure.wantedState = States.SHOOTER;
        }));

        driverController.y().onTrue(new InstantCommand(() ->
        {SwerveSubsystem.getInstance().io.zeroGyro();}));

        // Climb currently not implemented
        // driverController.b().onTrue(new SetStateClimbL3()); //remove climb
        // driverController.a().onTrue(new SetStateUnclimb());

        driverController.rightTrigger(0.5).onTrue(new SetStateShooter());
        driverController.rightTrigger(0.5).onTrue(new InstantCommand(
                () -> Shooter.driverOverride = true))
                .onFalse(new InstantCommand(
                        () -> Shooter.driverOverride = false));

        // coDriverController.b().onTrue(new SetStateManual());
        // coDriverController.leftTrigger(0.5).onTrue(new SetStateSafe());
        driverController.a().onTrue(new Spit());
        // coDriverController.rightTrigger(0.5).onTrue(new SetStateShooter());
        // coDriverController.rightTrigger().onTrue(new InstantCommand(
        //         () -> Shooter.driverOverride = true))
        //         .onFalse(new InstantCommand(
        //                 () -> Shooter.driverOverride = false));

        driverController.b().onTrue(new InstantCommand(
        () -> Superstructure.getInstance().wantedState = States.FIXEDFIRE))
        .onFalse(new InstantCommand(
                () -> Superstructure.getInstance().wantedState = States.SHOOTER));
        //X : TODO add unstuck
        driverController.rightBumper().onTrue(new InstantCommand(
        () -> Intake.getInstance().wantedState = IntakeStates.ELEPHANTIASISPART2))
        .onFalse(new InstantCommand(
                () -> Intake.getInstance().wantedState = IntakeStates.EXTENDED));
        //X : TODO add unstuck

        //LB : TODO add Reverse Intake

    }
}
