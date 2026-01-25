package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Climb.ClimbStates;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Feeder.Feeder;
import frc.robot.subsystems.Shooter.Feeder.FeederStates;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelStates;
import frc.robot.subsystems.Shooter.Turret.Turret;
import frc.robot.subsystems.Shooter.Turret.TurretStates;

public class Superstructure extends SubsystemBase{
    States currentState;
    States wantedState;

    @Override
    public void periodic() {
        handleStateTransitions();
        applyStates();
    }

    public void handleStateTransitions() {
        switch (wantedState) {
            case SAFE:
                break;
            case MANUAL:
                break;
            case CLIMB:
                break;
            case SHOOTER:
                break;
            case SHOOTERCON:
                break;
            default:
                break;
        }
    }

    public void applyStates() {
        switch (currentState) {
            case SAFE:
                stateSAFE();
                break;
            case MANUAL:
                stateMANUAL();
                break;
            case CLIMB:
                stateCLIMB();
                break;
            case SHOOTER:
                stateSHOOTER();
                break;
            case SHOOTERCON:
                stateSHOOTERCON();
                break;
            default:
                //do nothing
                break;
        }
    }

    private void stateSAFE() {
        Climb.getInstance().setClimbState(ClimbStates.IDLE);
    }

    private void stateMANUAL() {

    }

    private void stateCLIMB() {
        Climb.getInstance().setClimbState(ClimbStates.DEPLOYED);
    }

    private void stateSHOOTER(){

    }

    private void stateSHOOTERCON() {

    }
}
