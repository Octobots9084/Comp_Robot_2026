package frc.robot;

import java.lang.Thread.State;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotTypes;
import frc.robot.commands.auto.runIntake;
import frc.robot.commands.auto.ControllerInputs.Spit;
import frc.robot.commands.auto.StateChange.*;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Climb.ClimbStates;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;

public class ButtonConfig {
    public static CommandXboxController driverController = new CommandXboxController(0);
    public static CommandXboxController coDriverController = new CommandXboxController(1);
    public static Superstructure superstructure = Superstructure.getInstance();

    public void initTeleop() {
        SmartDashboard.putBoolean("A button", false);
        driverController.rightTrigger(0.5).onTrue(new InstantCommand(
                () -> Shooter.driverOverride = true))
                .onFalse(new InstantCommand(
                        () -> Shooter.driverOverride = false));
        // SmartDashboard.putBoolean("rightrigger",true);
        // driverController.rightTrigger(0.5).onTrue(new InstantCommand(
        // () -> {Shooter.getInstance().wantedShooterState = ShooterStates.HUB;
        // SmartDashboard.putBoolean("rightrigger",true);}))
        // .onFalse(new InstantCommand(
        // () -> {Shooter.getInstance().wantedShooterState = ShooterStates.SAFE;
        // /*superstructure.setWantedState(States.SAFE);*/
        // SmartDashboard.putBoolean("rightrigger",false);}
        // ));

        // driverController.rightBumper().onTrue(new InstantCommand( () -> {
        // Shooter.getInstance().turretAim = 0;
        // }));

        driverController.rightBumper().onTrue(new InstantCommand(() -> {
            Shooter.getInstance().turretAim = -0.57;
        }));

        // driverController.leftBumper().onTrue(new InstantCommand(() -> {
        // Shooter.getInstance().turretAim = -0.29;
        // }));
        driverController.leftBumper().onTrue(new InstantCommand(
                () -> {
                    Shooter.getInstance().wantedShooterState = ShooterStates.ZERO;
                    SmartDashboard.putBoolean("rightrigger", true);
                }))
                .onFalse(new InstantCommand(
                        () -> {
                            Shooter.getInstance().wantedShooterState = ShooterStates.HUB;
                            /* superstructure.setWantedState(States.SAFE); */ SmartDashboard.putBoolean("rightrigger",
                                    false);
                        }));
        // driverRight.button(1).onTrue(new InstantCommand(() ->
        // SwerveSubsystem.getInstance().io.zeroGyro()));

        driverController.y().onTrue(new InstantCommand(() -> {
            SwerveSubsystem.getInstance().io.zeroGyro();
            SmartDashboard.putBoolean("A button", true);
        })).onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("A button", false)));

        if (Constants.robotType != RobotTypes.ALPHA) {
            driverController.a().onTrue(new SetStateClimb());
            driverController.b().onTrue(new SetStateUnclimb());
            // driverController.leftBumper().whileTrue(new
            // SetIntakeStateIntaking()).onFalse(new SetIntakeStateSafe());
            driverController.leftTrigger(0.5).whileTrue(new SetIntakeStateIntaking()).onFalse(new SetIntakeStateSafe());
            // driverController.rightTrigger(0.5).whileTrue();

            coDriverController.b().onTrue(new SetStateManual());
            coDriverController.leftTrigger(0.5).onTrue(new SetStateSafe());
            coDriverController.leftBumper().onTrue(new SetStateSafe()); // yo twin, make ts cancel instead of safe state
                                                                        // -Oliver (trust)
            coDriverController.rightTrigger().onTrue(new Spit())
                    .onFalse(new InstantCommand(() -> {
                        if (Superstructure.getInstance().currentState == States.MANUAL) {
                            Shooter.driverOverride = false;
                        }
                    }));
        }
        // add in manual mode for turret and hood
    }
}
