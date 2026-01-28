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

    @Override
    public void periodic() {
        handleStateTransitions();
        applyStates();

        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
    }

    public void handleStateTransitions() {
        switch (wantedState) {
            case IDLE:
                //when the driver presses the idle button
               if (currentState == ClimbStates.DEPLOYED) {
                    currentState = ClimbStates.IDLE;
               }
                break;
            case CLIMBING:
                if (currentState == ClimbStates.GRABBING) {
                    currentState = ClimbStates.CLIMBING;
                } 
                break;
            case GRABBING:
                if (currentState == ClimbStates.DEPLOYED || currentState == ClimbStates.CLIMBING) {
                    currentState = ClimbStates.GRABBING;
                }
                break;
            case DEPLOYED:
                if (currentState == ClimbStates.IDLE || currentState == ClimbStates.GRABBING) {
                    currentState = ClimbStates.DEPLOYED;
                }
                break;
            default:
                
                break;
        }
    }

    public void applyStates()  {
        setClimbState(currentState); // I think
        // switch (currentState) {
        //     case IDLE:
        //         //to do nothing, wait fo wanted state
        //         break;
        //     case CLIMBING:
        //         //rotates,
        //         break;
        //     case GRABBING:
        //         //grab the bar go to climbing
        //         break;
        //     case DEPLOYED:
        //         //deploy the climb,go to grab,
        //         break;
        //     default:
        //         // this is bad i think
        //         break;
        // }
    }
}
