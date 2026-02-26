package frc.robot.subsystems;

import java.security.spec.ECPublicKeySpec;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.auto.StateChange.SetIntakeStateSafe;
import frc.robot.subsystems.*; // WHY DID WE HAVE SO MANY IMPORTS FROM THIS THING JUST IMPORT IT ALL
import frc.robot.subsystems.Climb.*; //I don't know why we need this
import frc.robot.subsystems.Intake.*;//same
import frc.robot.subsystems.Shooter.*;//same here
import frc.robot.subsystems.Shooter.Feeder.Feeder;
import frc.robot.subsystems.Shooter.Flywheel.*;

public class Superstructure extends SubsystemBase {
    public States currentState = States.SAFE;
    public States wantedState = States.SAFE;
    public IntakeStates userRequestedIntakeState = IntakeStates.SAFE;

    boolean climbDescending = true;
    boolean climbAligned = false;
    public static Superstructure currentInstance;
    // public Climb climb = Climb.getInstance();

    @Override
    public void periodic() {
        handleStateTransitions();
        applyStates();
    }

    public Superstructure() {
        currentInstance = this;
    }

    public static void setInstance(Superstructure instance) {
        currentInstance = instance;
    }

    public static Superstructure getInstance() {
        // if(currentInstance == null){
        // throw new IllegalStateException("Superstructure Instance not set");
        // }
        return currentInstance;
    }

    public States getCurrentState() {
        return currentState;
    }

    public States getWantedState() {
        return wantedState;
    }

    public void setCurrentState(States state) {
        currentState = state;
    }

    public void setWantedState(States state) {
        wantedState = state;
    }

    public void handleStateTransitions() {
        switch (wantedState) {
            case SAFE:
                // if (climb.getClimbState() != ClimbStates.CLIMBEDL1 || climb.getClimbState() != ClimbStates.CLIMBEDL3) {
                    currentState = States.SAFE;
                // }
            case CLIMB_L3:
                currentState = States.CLIMB_L3;
            case CLIMB_L1:
                currentState = States.CLIMB_L1;

            case SHOOTER:
                // if (climb.getClimbState() != ClimbStates.CLIMBEDL1 || climb.getClimbState() != ClimbStates.CLIMBEDL3) {
                    currentState = States.SHOOTER;
                // }
            case ZERO:
                currentState = States.ZERO;
                break;
            default:
                break; // do nothing
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
            case CLIMB_L3:
                stateCLIMBL3();
                break;
            case CLIMB_L1:
                stateCLIMBL1();
                break;
            case SHOOTER:
                // stateSHOOTER();
                Shooter.getInstance().wantedShooterState = ShooterStates.SPIT;
                break;
            case ZERO:
                // todo
                break;
            default:
                // throw an exception
                break;
        }
    }

    private void stateSAFE() {
        // Climb.getInstance().setClimbState(ClimbStates.IDLE);
        // set shooter into safe state
        // Intake.getInstance().setWantedState(IntakeStates.SAFE)

    }
    public void stowForClimb(){
        //   Intake.getInstance().setWantedState(IntakeStates.SAFE);
          Shooter.getInstance().wantedShooterState = ShooterStates.SAFE;
          
    }
    public boolean isClimbAligned() {
        if(climbAligned == true){
            return true;
        }else{
            return false;
        }
    
    }
    private void stateMANUAL() {
        // TODO map buttons to direct inputs
        // turn off intake when starting
        // turn off shooter when starting
    }
    private void stateCLIMBL3() {
        stowForClimb();
        // climb.setClimbState(ClimbStates.DEPLOYEDL3);
        // // TODO align to bar(use button before alignment)
        // climb.setClimbState(ClimbStates.ENGAGEDL3);
        // //TODO align to vertical pole(button before alignment)
        // climb.setClimbState(ClimbStates.CLIMBEDL3);
        boolean climbAligned = true; //TODO when rui finishes alignment put this when it finishes

    }
    private void stateCLIMBL1() {
        stowForClimb();
        // climb.setClimbState(ClimbStates.DEPLOYEDL1);
        // //TODO align to bar(button before alignment)
        // climb.setClimbState(ClimbStates.CLIMBEDL1);
    }
   

    private void stateSHOOTER() {
        // if (userRequestedIntakeState != Intake.getInstance().currentState) {
        //     Intake.getInstance().wantedState = userRequestedIntakeState;
        // }
    }
}