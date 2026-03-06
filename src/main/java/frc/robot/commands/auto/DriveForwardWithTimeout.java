package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.SwerveSubsystem;

public class DriveForwardWithTimeout extends Command {
    SwerveSubsystem swerve;
    double time;

    public DriveForwardWithTimeout(double time) {
        this.time = time;
        // SmartDashboard.putBoolean("did the thing", true);
    }

    public DriveForwardWithTimeout() {
        this.time = 0;
    }

    @Override
    public void initialize() {
        new DriveForward();// .withTimeout(time);
    }
}
