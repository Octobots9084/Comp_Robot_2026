package frc.robot.commands.auto;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.SwerveIO;
import frc.robot.subsystems.Drive.SwerveSubsystem;

public class DriveBack extends Command {
    @Override
    public void execute() {
        Logger.recordOutput("Test1", true);
    }   
}
