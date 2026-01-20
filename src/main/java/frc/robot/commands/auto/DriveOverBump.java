package frc.robot.commands.auto;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

public class DriveOverBump extends Command {
    SwerveSubsystem swerve;
    boolean onRamp;

    public DriveOverBump () {
            swerve = SwerveSubsystem.getInstance();
            onRamp = false;
        }//REMOVE WHEN HAVE THE GYRO FR we actualy need gyro inputs mb but maybe you can get from gyro
    @Override
    public void execute() {
        Logger.recordOutput("EXECUTING!!!!", true);
        
        //need Drive, Gyro (in Drive)

        //drive forward .3m //TODO: tune dist       wheelRadiusCharacterization

        new ParallelCommandGroup(
            new WaitCommand(2),
            new InstantCommand(() -> swerve.io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(new ChassisSpeeds(0.1, 0, 0))
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)))
        );
        

        onRamp = swerve.onRamp(0, 3);


        
        //
        if (onRamp) {
            swerve.io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(new ChassisSpeeds(0.1, 0, 0))
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
        }
    }

    // @Override
    // public boolean isFinished() {
    //     return !onRamp;
    // }
}
