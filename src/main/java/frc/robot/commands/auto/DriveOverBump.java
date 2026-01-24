package frc.robot.commands.auto;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

public class DriveOverBump extends Command {
    SwerveSubsystem swerve;
    boolean onRamp;
    boolean hasBeenOnRamp;

    public DriveOverBump () {
            swerve = SwerveSubsystem.getInstance();
            onRamp = false;
            hasBeenOnRamp = false;
        }//REMOVE WHEN HAVE THE GYRO FR we actualy need gyro inputs mb but maybe you can get from gyro

    @Override
    public void execute() {
        swerve.io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(new ChassisSpeeds(2, 0, 0))
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
        if (onRamp && !hasBeenOnRamp) {
            hasBeenOnRamp = true;
        }
    }

    @Override
    public boolean isFinished () {
        return (!onRamp && hasBeenOnRamp);
    }
}
