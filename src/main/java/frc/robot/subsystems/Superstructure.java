package frc.robot.subsystems;

import java.security.spec.ECPublicKeySpec;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.*; // WHY DID WE HAVE SO MANY IMPORTS FROM THIS THING JUST IMPORT IT ALL
import frc.robot.subsystems.Climb.*; //I don't know why we need this
import frc.robot.subsystems.Intake.*;//same
import frc.robot.subsystems.Shooter.*;//same here
import frc.robot.subsystems.Shooter.Flywheel.*;

public class Superstructure extends SubsystemBase{
    public States currentState = States.SAFE;
    public States wantedState = States.SAFE;
    
    boolean climbDescending = true;
    public static Superstructure currentInstance = null;
    public Climb climb = Climb.getInstance();


    @Override
    public void periodic() {
        handleStateTransitions();
        applyStates();
    }


    public Superstructure(){
        currentInstance = this;
    }

    public static void setInstance(Superstructure instance){
        currentInstance = instance;
    }

    public static Superstructure getInstance(){
        if(currentInstance == null){
            throw new IllegalStateException("Superstructure Instance not set");
        }
        return currentInstance;
    }
    public States getCurrentState(){
        return currentState;
    }

    public States getWantedState(){
        return wantedState;
    }

    public void setCurrentState(States state){
        currentState = state;
    }

    public void setWantedState(States state){
        wantedState = state;
    }
    public void handleStateTransitions() {
        switch(wantedState){
            case SAFE:
                if(climb.getClimbState() != ClimbStates.CLIMBEDL1 || climb.getClimbState() != ClimbStates.CLIMBEDL3){
                    currentState = States.SAFE;
                }
            case CLIMB:
                currentState = States.CLIMB;
            case SHOOTER:
                if(climb.getClimbState() != ClimbStates.CLIMBEDL1 || climb.getClimbState() != ClimbStates.CLIMBEDL3){
                    currentState = States.SHOOTER;
                }
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
            default:
                //throw an exception
                break;
        }
    }

    private void stateSAFE() {
        Climb.getInstance().setClimbState(ClimbStates.IDLE);
        //set shooter into safe state
        Intake.getInstance().setWantedState(IntakeStates.SAFE);

    }

    private void stateMANUAL() {
        //TODO map buttons to direct inputs
        //turn off intake when starting
        //turn off shooter when starting
    }

    private void stateCLIMB() {
        Intake.getInstance().setCurrentState(IntakeStates.SAFE);

        if(climb.climbL3){
            climb.setClimbState(ClimbStates.DEPLOYEDL3);
            // TODO align to bar(use button before alignment)
            climb.setClimbState(ClimbStates.ENGAGED);
            //TODO align to vertical pole(button before alignment)
            climb.setClimbState(ClimbStates.CLIMBEDL3);
        }else{
            climb.setClimbState(ClimbStates.DEPLOYEDL1);
            //TODO align to bar(button before alignment)
            climb.setClimbState(ClimbStates.CLIMBEDL1);
        }
       
    }

    private void stateSHOOTER(){
        //probably won't use, the transitions are run in the shooter
        // Shooter.getInstance().flywheel.setFlywheelVelocity(FlywheelStates.HUB); //TODO need to change this for variable speed
        
        // if (Shooter.getInstance().flywheel.getLeftMotorVelocity() >= 0.0){ //TODO set this to a speed
        //     // if (false /*manual override*/ || (true /*driver is asking to fire shooter*/ && ((Shooter.getInstance().inAllianceZone() && Shooter.getInstance().isHubActive()) || false /*ferrying*/))) {
        //     //     // scorePoints(PointsStates.A_LOT);
        //     // }
        //     if (Shooter.getInstance().inAllianceZone()) {
        //         Shooter.getInstance().hub();
        //     } else {
        //         //Shooter.getInstance().ferry();
        //     }
        // }
    }
}
























































































































































































































































































































































































































































































































































































































































































































































































































//