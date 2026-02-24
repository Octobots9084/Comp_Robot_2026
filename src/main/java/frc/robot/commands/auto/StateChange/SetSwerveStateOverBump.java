package frc.robot.commands.auto.StateChange;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeStates;

public class SetSwerveStateOverBump extends InstantCommand {
    public void SetSwerveStateBump() {
        SwerveSubsystem.getInstance().wantedState = SwerveSubsystem.SystemState.ROTATION_LOCK;
    }
}

