package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Subsystem; // do we need this?
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import things to check in handleStateTransitions()

public class Climb extends SubsystemBase {
    public ClimbStates currentState = ClimbStates.IDLE;
    public ClimbStates wantedState = ClimbStates.IDLE;
    public ClimbIO io;
    public boolean latched = false;
    public static Climb instance;
    public boolean alreadyZeroed = false;
    public ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    public Climb(ClimbIO io){
        this.io = io;
        instance = this;
    }

    public static Climb getInstance() {
        return instance;
    }

    public double getClimbPosition() {
        return io.getClimbPosition();
    }

    public boolean climbInTolerance(double climbTolerance) {
        return io.climbInTolerance(climbTolerance);
    }

    public void setClimbState(ClimbStates state) {
        io.setClimbState(state);
    }

    public ClimbStates getClimbState() {
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
        double currentPostition = currentState.climbPosition;
        switch (wantedState) {
            case IDLE:
                if (!latched) {
                    currentState = ClimbStates.IDLE;
                }
                break;
            case EXTENDED:
                if (currentState == ClimbStates.IDLE){
                    currentState = ClimbStates.EXTENDED;
                }
                break;
             case LATCHING:
                if(currentState == ClimbStates.EXTENDED && climbInTolerance(currentPostition)){
                    currentState = ClimbStates.LATCHING;
                }
                break;
            case CLIMBEDL1:
                if(currentState == ClimbStates.LATCHING && latched){
                    currentState = ClimbStates.CLIMBEDL1;
                }
                break;
            case UNCLIMB:
                if(currentState == ClimbStates.CLIMBEDL1){
                    currentState = ClimbStates.UNCLIMB;
                }
                break;
        }

    }

    public void applyStates() {

        switch(currentState){
            case EXTENDED:
            if(climbInTolerance(currentState.climbPosition)){
                latched = false;
            }
            setClimbState(currentState);
            break;
            case CLIMBEDL1:
                this.io.setCurrentLimit(60);
                setClimbState(currentState);
                break;
            case UNCLIMB:
                this.io.setCurrentLimit(20);
                setClimbState(currentState);
                break;
            case IDLE:
                setClimbState(currentState);
                break;
            case LATCHING:
                if(this.io.isAtCurrentLimit()){
                    latched = true;
                    wantedState = ClimbStates.CLIMBEDL1;
                }
                if(climbInTolerance(currentState.climbPosition)){
                    wantedState = ClimbStates.EXTENDED;
                }
                setClimbState(currentState);
                break;
        }
    }
}
