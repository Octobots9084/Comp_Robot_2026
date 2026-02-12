package frc.robot.commands.auto.NoPoseBump;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

public class DriveOverBumpFromAlliance extends Command {
    SwerveSubsystem swerve;
    boolean onRamp;
    boolean hasBeenOnRamp;
    double tilt;

    public DriveOverBumpFromAlliance () {
        swerve = SwerveSubsystem.getInstance();
        onRamp = false;
        hasBeenOnRamp = false;
        tilt = 0;
        SmartDashboard.putBoolean("test 1", hasBeenOnRamp);
    }
    
    @Override
    public void execute() {
        onRamp = swerve.onRamp(0, 0.3);
        move();
        if (onRamp && !hasBeenOnRamp) {
            hasBeenOnRamp = true;
        }
        SmartDashboard.putBoolean("test 1", hasBeenOnRamp);
        tilt = Math.acos(swerve.io.getRotation3d().toMatrix().get(2, 2)) - 0.015;
        SmartDashboard.putNumber("tilt", tilt);

    }// it is as expected. the is finished is returning early.
    //fix =? make a directional (when flat on peak of ramp, its !onRamp) -> TODO: fix -/+ for direction (advantageKit check which way is +)
    // 0 -> 1 -> 14 -> 15 -> 14 -> 12 -> 0 -> -1 -> -10 -> -15 -> -12 -> -3 -> 0 //TILT

    @Override
    public boolean isFinished () {
        return (!onRamp && hasBeenOnRamp && (tilt < -1));//TODO: here from other todo (-1 or 1)
        // return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        onRamp = false;
        hasBeenOnRamp = false;
        tilt = 0;
    }

    public void move () {
        if (Constants.allianceColor == Alliance.Blue) {
            swerve.io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(new ChassisSpeeds(.5, 0, 0))
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
        } else {
            swerve.io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(new ChassisSpeeds(-.5, 0, 0))
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
        }
    }

}

