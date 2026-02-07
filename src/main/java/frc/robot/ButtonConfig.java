package frc.robot;

import java.lang.Thread.State;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.runIntake;
import frc.robot.commands.auto.ControllerInputs.SebRightTrigger;
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
    public static CommandXboxController sebController = new CommandXboxController(1);
    public void initTeleop(){
    SmartDashboard.putBoolean("A button", false);
        driverController.rightTrigger().onTrue(new InstantCommand(
            () -> Shooter.driverOverride = true))
            .onFalse(new InstantCommand(
            () -> Shooter.driverOverride = false
            ));
        //driverRight.button(1).onTrue(new InstantCommand(() -> SwerveSubsystem.getInstance().io.zeroGyro()));
        driverController.a().onTrue(new SetStateClimb());
        driverController.b().onTrue(new SetStateUnclimb());
        
       
        driverController.y().onTrue(new InstantCommand(() -> {SwerveSubsystem.getInstance().io.zeroGyro(); SmartDashboard.putBoolean("A button", true);})).onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("A button", false)));
        
        driverController.leftBumper().whileTrue(new SetIntakeStateIntaking()).onFalse(new SetIntakeStateSafe());
        driverController.leftTrigger(0.5).whileTrue(new SetIntakeStateIntaking()).onFalse(new SetIntakeStateSafe());
        //driverController.rightTrigger(0.5).whileTrue();

        sebController.b().onTrue(new SetStateManual());
        sebController.leftTrigger(0.5).onTrue(new SetStateSafe());
        sebController.leftBumper().onTrue(new SetStateSafe()); //yo twin, make ts cancel instead of safe state -Oliver (trust)
        driverController.rightTrigger().onTrue(new SebRightTrigger())
            .onFalse(new InstantCommand(() -> {if (Superstructure.getInstance().currentState == States.MANUAL) {Shooter.driverOverride = false;}} ));
    }

    // public Command runIntake(){}
}
