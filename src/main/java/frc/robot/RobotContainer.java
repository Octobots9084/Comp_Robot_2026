// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Subsystems.Drive.Swerve;
import frc.robot.Subsystems.Drive.TunerConstants;
import frc.robot.Subsystems.Drive.SwerveIO;
import frc.robot.Subsystems.Drive.SwerveIOSystem;

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
    private final Swerve swerveSubsystem;
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
        this.swerveSubsystem = Swerve.setInstance(TunerConstants.createDrivetrain(), driverLeft, driverRight, Constants.maxAngularVelocity, Constants.maxVelocity);
        switch (Constants.currentMode) {
            case REAL:
                break;
            case SIM:
                break;
            default:
                break;
        }

        // autoChooser = AutoBuilder.buildAutoChooser();
        //NAMED COMMANDS IN SWERVE
        // SmartDashboard.putData("Autonomous Path", autoChooser);
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
        // return autoChooser.getSelected();
        return new InstantCommand();
    }
}