package frc.robot.commands.auto;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SwerveIO;
import frc.robot.subsystems.drive.SwerveSubsystem;
    
public class DriveBack extends Command {
    SwerveSubsystem swerve;

    @Override
    public void initialize () {
        swerve = SwerveSubsystem.getInstance();
    }

    @Override
    public void execute() {
        swerve.io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(new ChassisSpeeds(0.5, 0, 0)).withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
    }   
}
