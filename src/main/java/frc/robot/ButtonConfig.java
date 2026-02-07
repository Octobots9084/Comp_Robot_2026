package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.runIntake;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Shooter.Shooter;

public class ButtonConfig {
    public static CommandXboxController driverController = new CommandXboxController(0);
    public void initTeleop(){
    SmartDashboard.putBoolean("A button", false);
        driverController.rightTrigger(0.5).onTrue(new InstantCommand(
            () -> Shooter.driverOverride = true))
            .onFalse(new InstantCommand(
            () -> Shooter.driverOverride = false
            ));
        //driverRight.button(1).onTrue(new InstantCommand(() -> SwerveSubsystem.getInstance().io.zeroGyro()));
        driverController.a().onTrue(new InstantCommand(() -> {SwerveSubsystem.getInstance().io.zeroGyro(); SmartDashboard.putBoolean("A button", true);})).onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("A button", false)));
        // driverController.b().onTrue(new InstantCommand(() -> {Intake.getInstance().wantedState = IntakeStates.INTAKING; SmartDashboard.putBoolean("B button", true);})).onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("B button", false)));
        driverController.leftBumper().onTrue(new InstantCommand(()->{Intake.getInstance().wantedState = IntakeStates.REVERSEINTAKING;})).onFalse(new InstantCommand(()->{Intake.getInstance().wantedState = IntakeStates.SAFE;}));
        driverController.leftTrigger(0.5).onTrue(new InstantCommand(()->{Intake.getInstance().wantedState = IntakeStates.INTAKING;})).onFalse(new InstantCommand(()->{Intake.getInstance().wantedState = IntakeStates.SAFE;}));
    }
}
