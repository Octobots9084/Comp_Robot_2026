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
import frc.robot.subsystems.Drive.SwerveStates;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;

public class ButtonConfig {
    public static CommandXboxController driverController = new CommandXboxController(0);
    // public static CommandXboxController coDriverController = new
    // CommandXboxController(1);
    public Superstructure superstructure = Superstructure.getInstance();

    public void initTeleop() {
        SmartDashboard.putBoolean("A button", false);
        driverController.rightTrigger(0.5).onTrue(new InstantCommand(
                () -> Shooter.driverOverride = true))
                .onFalse(new InstantCommand(
                        () -> Shooter.driverOverride = false));
        driverController.leftBumper().onTrue(new InstantCommand(
                () -> {
                    superstructure.wantedState = States.ZERO;
                    SmartDashboard.putBoolean("rightrigger", true);
                }))
                .onFalse(new InstantCommand(
                        () -> {
                            superstructure.wantedState = States.SHOOTER;
                            /* superstructure.setWantedState(States.SAFE); */ SmartDashboard.putBoolean("rightrigger",
                                    false);
                        }));

        driverController.y().onTrue(new InstantCommand(() -> {
            SwerveSubsystem.getInstance().io.zeroGyro();
            SmartDashboard.putBoolean("A button", true);
        })).onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("A button", false)));

        driverController.x().onTrue(new InstantCommand(
                () -> {
                    SwerveSubsystem.getInstance().wantedState = SwerveStates.ALIGNCLIMB;
                    SmartDashboard.putBoolean("X", true);
                }))
                .onFalse(new InstantCommand(
                        () -> {
                            SwerveSubsystem.getInstance().wantedState = SwerveStates.MANUAL;
                            /* superstructure.setWantedState(States.SAFE); */ SmartDashboard.putBoolean("X", false);
                        }));
        driverController.leftBumper().whileTrue(new SetIntakeStateReverse()).onFalse(new SetIntakeStateSafe());
        driverController.leftTrigger().whileTrue(new SetIntakeStateIntaking()).onFalse(new SetIntakeStateSafe());
    }
}
