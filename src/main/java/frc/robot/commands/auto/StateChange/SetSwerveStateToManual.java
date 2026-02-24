package frc.robot.commands.auto.StateChange;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Drive.SwerveSubsystem;

public class SetSwerveStateToManual extends InstantCommand {
    public void SetSwerveStateManual() {
        SwerveSubsystem.getInstance().wantedState = SwerveSubsystem.SystemState.MANUAL;
    }
}

