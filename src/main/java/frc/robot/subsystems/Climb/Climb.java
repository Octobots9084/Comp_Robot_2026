package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Subsystem; // do we need this?
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import things to check in handleStateTransitions()


public class Climb extends SubsystemBase{
    ClimbStates currentState = ClimbStates.IDLE;
    ClimbStates wantedState = ClimbStates.IDLE;
    public ClimbIO io;
    public static Climb instance;
    public ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
    public boolean climbL3 = false;

    public Climb(ClimbIO io){
        this.io = io;
        instance = this;
    }

    public static Climb getInstance(){
        return instance;
    }

    public double getClimbPosition(){
        return io.getClimbPosition();
    }
    public double getDeployPosition(){
        return io.getDeployPosition();
    }
    public boolean climbInTolerance(double climbTolerance){
        return io.climbInTolerance(climbTolerance);
    }
    public void setClimbState(ClimbStates state){
        io.setClimbState(state);
    }

    public ClimbStates getClimbState(){
        return currentState;
    }

    @Override
    public void periodic() {
        handleStateTransitions();
        applyStates();

        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
    }

    public void handleStateTransitions() {
        switch (wantedState){
            case IDLE:
                if(currentState == ClimbStates.DEPLOYEDL1 || currentState == ClimbStates.DEPLOYEDL3){
                    currentState = ClimbStates.IDLE;
                }
            case DEPLOYEDL1:
                if(currentState == ClimbStates.IDLE || currentState == ClimbStates.CLIMBEDL1){
                    currentState = ClimbStates.DEPLOYEDL1;
                }
            case CLIMBEDL1:
                if(currentState == ClimbStates.DEPLOYEDL1){
                    currentState = ClimbStates.CLIMBEDL1;
                }
            case DEPLOYEDL3:
                if(currentState == ClimbStates.IDLE || currentState == ClimbStates.ENGAGED){
                    currentState = ClimbStates.DEPLOYEDL3;
                }
            case ENGAGED:
                if(currentState == ClimbStates.DEPLOYEDL3 || currentState == ClimbStates.CLIMBEDL3){
                    currentState = ClimbStates.ENGAGED;
                }
            case CLIMBEDL3:
                if(currentState == ClimbStates.ENGAGED){
                    currentState = ClimbStates.CLIMBEDL3;
                }
        }

    }

    public void applyStates()  {
        setClimbState(currentState);
    }
}
