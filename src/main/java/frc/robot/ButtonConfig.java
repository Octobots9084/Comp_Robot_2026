package frc.robot;


import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.auto.DriveBack;
import frc.robot.commands.auto.DriveOverBump;
import frc.robot.subsystems.Drive.SwerveSubsystem;
public class ButtonConfig {
    static CommandJoystick driverLeft = ControlMap.DRIVER_LEFT;
    static CommandJoystick driverRight = ControlMap.DRIVER_RIGHT;
    static CommandJoystick driverButtons = ControlMap.DRIVER_BUTTONS;
    static CommandJoystick coDriverLeft = ControlMap.CO_DRIVER_LEFT;
    static CommandJoystick coDriverRight = ControlMap.CO_DRIVER_RIGHT;
    static CommandJoystick coDriverButtons = ControlMap.CO_DRIVER_BUTTONS;

    public void initTeleop() {
        driverRight.button(1).onTrue(new InstantCommand(() -> SwerveSubsystem.getInstance().io.zeroGyro()));
    }
}
