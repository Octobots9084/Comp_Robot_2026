package frc.robot.commands.auto;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.SwerveSubsystem;

public class DriveForwardUntilLevel extends Command {
    SwerveSubsystem swerve;

    @Override
    public void initialize() {
        swerve = SwerveSubsystem.getInstance();
    }

    @Override
    public void execute() {
        if (Constants.isBlueAlliance) {
            swerve.io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(new ChassisSpeeds(0, -2, 0))
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
        } else {
            swerve.io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(new ChassisSpeeds(0, 2, 0))
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
        }
    }

    @Override
    public boolean isFinished() {
        // SmartDashboard.putBoolean("FinishedDriveForwardUntilLevel", !swerve.onRamp(0, 2));
        return !swerve.onRamp(0, 0.1);
    }
}
