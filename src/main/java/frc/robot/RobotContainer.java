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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIOTalonFX;
import frc.robot.Constants.RobotTypes;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Climb.ClimbIOTalonFX;
import frc.robot.subsystems.Drive.BetaConstants;
import frc.robot.subsystems.Drive.MangoConstants;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Feeder.FeederIOTalonFX;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.Shooter.Turret.TurretIOTalonFX;
import frc.robot.subsystems.Vision.Vision;

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
  public Shooter shooter;
  public Intake intake;
  public Climb climb;
  public Vision vision;
  private SwerveSubsystem swerve;
  private Superstructure superstructure;
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final SendableChooser<Command> autoChooser;
    
  // private final SendableChooser<Command> autoChooser;
  static CommandJoystick driverLeft = ControlMap.DRIVER_LEFT;
  static CommandJoystick driverRight = ControlMap.DRIVER_RIGHT;
  static CommandJoystick driverButtons = ControlMap.DRIVER_BUTTONS;
  static CommandJoystick coDriverLeft = ControlMap.CO_DRIVER_LEFT;
  static CommandJoystick coDriverRight = ControlMap.CO_DRIVER_RIGHT;
  static CommandJoystick coDriverButtons = ControlMap.CO_DRIVER_BUTTONS;
  // The robot's subsystems and commands are defined here...

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
              //TODO change the buttons from driverleft and right to the xbox controller
              if (Constants.robotType == RobotTypes.ALPHA) {
                this.swerve = SwerveSubsystem.setInstance(MangoConstants.createDrivetrain(), ButtonConfig.driverController, Constants.maxAngularVelocity, Constants.maxVelocity);
              } else {
                this.swerve = SwerveSubsystem.setInstance(BetaConstants.createDrivetrain(), ButtonConfig.driverController, Constants.maxAngularVelocity, Constants.maxVelocity);
              }
              shooter = new Shooter(
                new FeederIOTalonFX(), 
                new FlywheelIOTalonFX(), 
                new TurretIOTalonFX());
              intake = new Intake(new IntakeIOTalonFX());
              climb = new Climb(new ClimbIOTalonFX());
                break;
            case SIM:
              break;
            default:
              if (Constants.robotType == RobotTypes.ALPHA) {
                this.swerve = SwerveSubsystem.setInstance(MangoConstants.createDrivetrain(), ButtonConfig.driverController, Constants.maxAngularVelocity, Constants.maxVelocity);
              } else {
                this.swerve = SwerveSubsystem.setInstance(BetaConstants.createDrivetrain(), ButtonConfig.driverController, Constants.maxAngularVelocity, Constants.maxVelocity);
              }
              break;
        }

        autoChooser = AutoBuilder.buildAutoChooser();
        //NAMED COMMANDS IN SWERVE
       SmartDashboard.putData("Auto", autoChooser);
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