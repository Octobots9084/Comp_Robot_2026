package frc.robot.commands.auto;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.SwerveSubsystem;

public class DriveOverBump extends Command {
    SwerveSubsystem swerve;
    boolean onRamp;
    boolean hasBeenOnRamp;

    public DriveOverBump() {
        swerve = SwerveSubsystem.getInstance();
        onRamp = false;
        hasBeenOnRamp = false;
    }

    @Override
    public void execute() {
        onRamp = swerve.onRamp(0, 0.1);
        move();
        if (onRamp && !hasBeenOnRamp) {
            hasBeenOnRamp = true;
        }
    }

    @Override
    public boolean isFinished() {
        return (!onRamp && hasBeenOnRamp);
        // return false;
    }

    @Override
    public void end(boolean interrupted) {
        onRamp = false;
        hasBeenOnRamp = false;
    }

    public void move() {
        if (Constants.allianceColor == Alliance.Blue) {
            swerve.io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(new ChassisSpeeds(2, 0, 0))
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
        } else {
            swerve.io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(new ChassisSpeeds(-2, 0, 0))
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
        }
    }

}
