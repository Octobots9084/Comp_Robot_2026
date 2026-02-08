// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIOTalonFX;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Climb.ClimbIOTalonFX;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Feeder.Feeder;
import frc.robot.subsystems.Shooter.Feeder.FeederIOTalonFX;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.Shooter.Turret.TurretIOTalonFX;
import frc.robot.subsystems.Drive.SwerveIO;
import frc.robot.subsystems.Drive.SwerveIOSystem;
import frc.robot.subsystems.Drive.SwerveSubsystem;

import frc.robot.Constants.RobotTypes;
import frc.robot.subsystems.Drive.AlphaConstants;
import frc.robot.subsystems.Drive.BetaConstants;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private Shooter shooter;
  private Intake intake;
  private Climb climb;
  private SwerveSubsystem swerve;
  private Superstructure superstructure;
  // Controller
  private CommandXboxController controller;

  // Dashboard inputs
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //TODO change the buttons from driverleft and right to the xbox controller
    if (Constants.robotType == RobotTypes.BETA) {
            this.swerve = SwerveSubsystem.setInstance(BetaConstants.createDrivetrain(), ButtonConfig.driverController, Constants.maxAngularVelocity, Constants.maxVelocity);
        } else {
            this.swerve = SwerveSubsystem.setInstance(AlphaConstants.createDrivetrain(), ButtonConfig.driverController, Constants.maxAngularVelocity, Constants.maxVelocity);
        }
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        if (Constants.robotType != RobotTypes.ALPHA){
          shooter = new Shooter(
              new FeederIOTalonFX(), 
              new FlywheelIOTalonFX(), 
              new TurretIOTalonFX(),
              ButtonConfig.coDriverController);
          intake = new Intake(new IntakeIOTalonFX());
          climb = new Climb(new ClimbIOTalonFX());
          superstructure = new Superstructure();
      }


        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:

        shooter = new Shooter(new FeederIOTalonFX(), new FlywheelIOTalonFX(), new TurretIOTalonFX(), ButtonConfig.coDriverController);
        intake = new Intake(new IntakeIOTalonFX());
        climb = new Climb(new ClimbIOTalonFX());
        superstructure = new Superstructure();
        break;

      default:
        // Replayed robot, disable IO implementations
    }
        autoChooser = AutoBuilder.buildAutoChooser();
        //NAMED COMMANDS IN SWERVE
       SmartDashboard.putData("Auto", autoChooser);
        // VisionSubsystem.getInstance();
        ButtonConfig buttons = new ButtonConfig();
        buttons.initTeleop();
    }

    public SwerveSubsystem getSwerveSubsystem () {
        return swerve;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
        // return new InstantCommand();
    }
}