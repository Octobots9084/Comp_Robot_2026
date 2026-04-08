package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.runIntake;
import frc.robot.commands.auto.runIntakeReverse;
import frc.robot.commands.auto.ControllerInputs.Spit;
import frc.robot.commands.auto.StateChange.*;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Drive.SwerveStates;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Lights.LightAnimations;
import frc.robot.subsystems.Lights.Lights;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;

public class ButtonConfig {
    public static CommandXboxController driverController = new CommandXboxController(0);
    public static CommandXboxController coDriverController = new CommandXboxController(1);
    public Superstructure superstructure = Superstructure.getInstance();

    public void initTeleop() {

        
        // driverController.leftTrigger().onTrue(new SetIntakeStateIntaking()).onFalse(new SetIntakeStateSafe());

        driverController.rightBumper().whileTrue(new runIntakeReverse());
        driverController.leftTrigger().whileTrue(new runIntake());


        driverController.x().onTrue(new InstantCommand(() -> {
                superstructure.wantedState = States.UNJAM;
        })).onFalse(new InstantCommand(() -> {
                superstructure.wantedState = States.SHOOTER;
        }));

        driverController.b().onTrue(new InstantCommand(() -> {
                                                                if (SwerveSubsystem.getInstance().wantedState == SwerveStates.ROTATION_LOCK) {
                                                                        SwerveSubsystem.getInstance().wantedState = SwerveStates.MANUAL;
                                                                } else {
                                                                        SwerveSubsystem.getInstance().wantedState = SwerveStates.ROTATION_LOCK;
                                                                }
                                                        }));

        // coDriverController.a().onTrue(new InstantCommand(() ->
        // {SwerveSubsystem.getInstance().io.zeroGyro();}));
        coDriverController.a().onTrue(new InstantCommand(() ->
        {Shooter.getInstance().manuelHood+=0.25;}));
        coDriverController.b().onTrue(new InstantCommand(() ->
        {Shooter.getInstance().manuelHood-=0.25;}));
        coDriverController.x().onTrue(new InstantCommand(() ->
        {Shooter.getInstance().manuelFlywheel+=0.25;}));
        coDriverController.y().onTrue(new InstantCommand(() ->
        {Shooter.getInstance().manuelFlywheel-=0.25;}));

        // Climb currently not implemented
        // driverController.b().onTrue(new SetStateClimbL3()); //remove climb
        // driverController.a().onTrue(new SetStateUnclimb());

        driverController.rightTrigger(0.5).onTrue(new SetStateShooter());
        driverController.rightTrigger(0.5).onTrue(new InstantCommand(
                () -> Shooter.driverOverride = true))
                .onFalse(new InstantCommand(
                        () -> Shooter.driverOverride = false));

        coDriverController.b().onTrue(new SetStateManual());
        //coDriverController.leftTrigger(0.5).onTrue(new SetStateSafe());
        driverController.a().onTrue(new Spit());
        driverController.y().onTrue(new InstantCommand(() -> superstructure.wantedState = States.FIXEDFIRE)).onFalse(new InstantCommand(() -> superstructure.wantedState = States.SHOOTER));
        coDriverController.rightTrigger(0.5).onTrue(new SetStateShooter());
        coDriverController.rightTrigger().onTrue(new InstantCommand(
                () -> Shooter.driverOverride = true))
                 .onFalse(new InstantCommand(
                         () -> Shooter.driverOverride = false));

        //X : TODO add unstuck
        // driverController.rightBumper().onTrue(new InstantCommand(
        // () -> Intake.getInstance().wantedState = IntakeStates.ELEPHANTIASISPART2))
        // .onFalse(new InstantCommand(
        //         () -> Intake.getInstance().wantedState = IntakeStates.EXTENDED));
        //X : TODO add unstuck

        // driverController.rightBumper().whileTrue(new InstantCommand(() -> {
        //         superstructure.wantedState = States.SPITTOCONTAINER;
        // })).onFalse(new InstantCommand(() -> {
        //         superstructure.wantedState = States.SHOOTER;
        // }));

        // driverController.povDownRight().onTrue(new InstantCommand(() -> {
        //         Shooter.getInstance().turret.io.moveTurretAndHoodToZero();
        //         Shooter.getInstance().wantedShooterState = ShooterStates.SAFE;
        // })).onFalse(new InstantCommand(() -> {
        //         Shooter.getInstance().wantedShooterState = ShooterStates.HUB;
        // }));
        driverController.leftBumper().whileTrue(new InstantCommand(() -> {
                Intake.getInstance().setWantedState(IntakeStates.ELEPHANTIASISPART2);
        })).onFalse(new InstantCommand(() -> {
                        Intake.getInstance().setWantedState(IntakeStates.EXTENDED);

        }));

        if(Shooter.getInstance().currentShooterState != ShooterStates.MANUEL){
                driverController.povUp().onTrue(new InstantCommand( () -> {
                        Shooter.getInstance().wantedShooterState = ShooterStates.MANUEL;
                }));
        }else{
                driverController.povUp().onTrue(new InstantCommand( () -> {
                        Shooter.getInstance().wantedShooterState = ShooterStates.HUB;
                }));
        }


        //TODO remove this after testing
        // driverController.y().onTrue(new InstantCommand(() -> {
        //         LightAnimations anim = Lights.getLightInstance().lightsCurrentState;
        //         LightAnimations target = LightAnimations.INTAKING;

        //         if (anim == LightAnimations.INTAKING) target = LightAnimations.CANTSHOOT;
        //         if (anim == LightAnimations.CANTSHOOT) target = LightAnimations.SHOOTFERRY;
        //         if (anim == LightAnimations.SHOOTFERRY) target = LightAnimations.SHOOTHUB;
        //         if (anim == LightAnimations.SHOOTHUB) target = LightAnimations.INTAKING;

        //         if (target != LightAnimations.INTAKING) Lights.getLightInstance().lightsCurrentState = target;
        // }));

    }
}
