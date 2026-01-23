package frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase{
    ClimbStates currentState = ClimbStates.IDLE;
    ClimbStates wantedState = ClimbStates.IDLE;
    public ClimbIO io = new ClimbIOTalonFX();


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
    }

    public void handleStateTransitions() {
        switch (wantedState) {
            case IDLE:
               //when the driver presses the idle button
                break;
            case CLIMBING:
                
                break;
            case GRABBING:
                
                break;
            case DEPLOYED:
                
                break;
            default:
                
                break;
        }
    }

    public void applyStates() {
        io.setClimbState(currentState);

        switch (currentState) {
            case IDLE:
                //to do nothing, wait fo wanted state
                break;
            case CLIMBING:
                //rotates,
                break;
            case GRABBING:
                //grab the bar go to climbing
                break;
            case DEPLOYED:
                //deploy the climb,go to grab,
                break;
            default:
                // this is bad i think
                break;
        }
    }
}
