// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIOTalonFX;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Climb.ClimbIOTalonFX;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Feeder.FeederIOTalonFX;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.Shooter.Turret.TurretIOTalonFX;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.drive.TunerConstants;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;


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
    // private final SendableChooser<Command> autoChooser;
    // Subsystems
    public Shooter shooter;
    public Intake intake;
    public Climb climb;
    public Vision vision;

    private final SwerveSubsystem swerveSubsystem;
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
        // creates objects for the robot subsystems
        this.swerveSubsystem = SwerveSubsystem.setInstance(TunerConstants.createDrivetrain(), driverLeft, driverRight, Constants.maxAngularVelocity, Constants.maxVelocity);
        vision = new Vision();
        shooter = new Shooter(
            new FeederIOTalonFX(), 
            new FlywheelIOTalonFX(), 
            new TurretIOTalonFX());
        intake = new Intake(new IntakeIOTalonFX());
        climb = new Climb(new ClimbIOTalonFX());
        switch (Constants.currentMode) {
            case REAL:
                break;
            case SIM:
                break;
            default:
                break;
        }

        //autoChooser = AutoBuilder.buildAutoChooser();
        //NAMED COMMANDS IN SWERVE
      //  SmartDashboard.putData("Autonomous Path", autoChooser);
        // VisionSubsystem.getInstance();
        ButtonConfig buttons = new ButtonConfig();
        buttons.initTeleop();
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}