package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.runIntake;
import frc.robot.commands.auto.runIntakeReverse;
import frc.robot.commands.auto.ControllerInputs.Spit;
import frc.robot.commands.auto.ControllerInputs.ToggleAutoFerry;
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

    public Intake intake;
//     public static CommandXboxController coDriverController = new CommandXboxController(1);
    public Superstructure superstructure = Superstructure.getInstance();

    public void initTeleop() {
        intake = Intake.getInstance();
        
        // driverController.leftTrigger().onTrue(new SetIntakeStateIntaking()).onFalse(new SetIntakeStateSafe());

        driverController.rightBumper().whileTrue(new runIntakeReverse());
        driverController.leftTrigger().onTrue(new InstantCommand(() -> {Intake.driverOverride = true;Intake.getInstance().wantedState=IntakeStates.INTAKING;})).onFalse(new InstantCommand(() -> {Intake.driverOverride = false;Intake.getInstance().wantedState=IntakeStates.EXTENDED;}));
        // driverController.leftTrigger().onTrue(new InstantCommand(() -> {Intake.driverOverride = true;})).onFalse(new InstantCommand(() -> {Intake.driverOverride = false;}));

        driverController.x().onTrue(new InstantCommand(() -> {
                superstructure.wantedState = States.UNJAM;
        })).onFalse(new InstantCommand(() -> {
                superstructure.wantedState = States.SHOOTER;
        }));

        // driverController.b().onTrue(new InstantCommand(() -> {
        //         if (SwerveSubsystem.getInstance().wantedState == SwerveStates.ROTATION_LOCK) {
        //                 SwerveSubsystem.getInstance().wantedState = SwerveStates.MANUAL;
        //         } else {
        //                 SwerveSubsystem.getInstance().wantedState = SwerveStates.ROTATION_LOCK;
        //         }
        // }));

        // coDriverController.a().onTrue(new InstantCommand(() ->
        // {SwerveSubsystem.getInstance().io.zeroGyro();}));
        // coDriverController.a().onTrue(new InstantCommand(() ->
        // {Shooter.getInstance().manuelHood+=0.25;}));
        // coDriverController.b().onTrue(new InstantCommand(() ->
        // {Shooter.getInstance().manuelHood-=0.25;}));
        // coDriverController.x().onTrue(new InstantCommand(() ->
        // {Shooter.getInstance().manuelFlywheel+=0.25;}));
        // coDriverController.y().onTrue(new InstantCommand(() ->
        // {Shooter.getInstance().manuelFlywheel-=0.25;}));
        //TODO add back
        driverController.rightTrigger(0.5).onTrue(new SetStateShooter());
        driverController.rightTrigger(0.5).onTrue(new InstantCommand(
                () -> Shooter.driverOverride = true))
                .onFalse(new InstantCommand(
                        () -> Shooter.driverOverride = false)).onFalse(new InstantCommand(() -> {Shooter.flywheelDebouncer = Shooter.flywheelToleranceThreshold;}));

    
        coDriverController.x().onTrue(new InstantCommand(() -> Superstructure.getInstance().wantedState = States.ZERO));
        coDriverController.rightTrigger(0.5).onTrue(new SetStateShooter());
        coDriverController.rightTrigger(0.5).onTrue(new InstantCommand(
                () -> Shooter.driverOverride = true))
                .onFalse(new InstantCommand(
                        () -> Shooter.driverOverride = false)).onFalse(new InstantCommand(() -> {Shooter.flywheelDebouncer = Shooter.flywheelToleranceThreshold;}));

        driverController.a().onTrue(new ToggleAutoFerry());

        coDriverController.b().onTrue(new InstantCommand(
                () -> {
                        Shooter.flywheelOverride = true;
                })).onFalse(new InstantCommand(() -> {
                        Shooter.flywheelOverride = false;
                }));
        driverController.povUp().onTrue(new InstantCommand(
                () -> {
                        Shooter.flywheelOverride = true;
                })).onFalse(new InstantCommand(() -> {
                        Shooter.flywheelOverride = false;
                }));

        driverController.leftBumper().onTrue( new InstantCommand( () -> {
                intake.wantedState = IntakeStates.ELEPHANTIASISPART2;
                //override is cringe lmaooo
                // Intake.driverElephantiasisPart2Override = true;
        })).onFalse( new InstantCommand( () -> {
                // Intake.driverElephantiasisPart2Override = false;
        }));
        coDriverController.a().onTrue(new ToggleAutoFerry());
        
        driverController.b().onTrue(new InstantCommand(() -> {
                intake.wantedState = IntakeStates.SAFE;
        }));

        //coDriverController.leftTrigger(0.5).onTrue(new SetStateSafe());
        driverController.y().onTrue(new InstantCommand(() -> superstructure.wantedState = States.FIXEDFIRE)).onFalse(new InstantCommand(() -> superstructure.wantedState = States.SHOOTER));
        

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



        // driverController.leftBumper().onTrue(new InstantCommand(() -> {
        //         Intake.getInstance().setWantedState(IntakeStates.ELEPHANTIASISPART2);
        // })).onFalse(new InstantCommand(() -> {
        //                 Intake.getInstance().setWantedState(IntakeStates.EXTENDED);

        // }));//UNCOMMENT AFTER AUSTION TESTING OF INTAKE (its the real one)

        // driverController.leftBumper().onTrue(new InstantCommand(() -> {
        //         Intake.getInstance().setWantedState(IntakeStates.ELEPHANTIASISPART2);
        // }));//for braking tha motors


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
