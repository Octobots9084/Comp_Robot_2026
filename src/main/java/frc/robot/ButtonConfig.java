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
    public static CommandXboxController coDriverController = new CommandXboxController(1);
    public Superstructure superstructure = Superstructure.getInstance();

    public void initTeleop() {
        SmartDashboard.putBoolean("A button", false);

        
        driverController.leftTrigger().onTrue(new SetIntakeStateIntaking()).onFalse(new SetIntakeStateSafe());

        driverController.leftBumper().onTrue(new SetIntakeStateReverse()).onFalse(new SetIntakeStateSafe());

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
        coDriverController.leftTrigger(0.5).onTrue(new SetStateSafe());
        coDriverController.rightBumper().onTrue(new Spit()).onFalse(new SetStateShooter());
        driverController.rightTrigger(0.5).onTrue(new SetStateShooter());
        coDriverController.rightTrigger().onTrue(new InstantCommand(
                () -> Shooter.driverOverride = true))
                .onFalse(new InstantCommand(
                        () -> Shooter.driverOverride = false));

    }
}
