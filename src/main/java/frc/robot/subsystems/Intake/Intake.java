package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    
    IntakeStates currentState = IntakeStates.SAFE;
    IntakeStates wantedState = IntakeStates.SAFE;

    /*
    Motor liftMotor = new Motor();
    Motor spinMotor = new Motor();
    */
    
    
    @Override
    public void periodic() {
        //This will handle changing between states at the user's request.
        handleStateTransitions();

        //this is where states actually take effect.
        applyStates();
    }
    public void handleStateTransitions() {
    
        switch(wantedState) {
            case SAFE:
            currentState = IntakeStates.SAFE;
            break;

            case INTAKING:
                //only works if not climbing
            currentState = IntakeStates.INTAKING;
            break;

            case EXTENDED:
                //only works if not climbing
            currentState = IntakeStates.EXTENDED;
            break;

            case REVERSEINTAKING:
                //only works if not climbing
            currentState = IntakeStates.REVERSEINTAKING;
            break;

            default:
            currentState = IntakeStates.SAFE;
            break;
            //67

        }
    }

    public void applyStates() {
        
        switch (currentState){

        case INTAKING:
        //motors on intake out
        break;

        case EXTENDED:
        //motors off intake out
        break;

        case SAFE:
        //motors off intake in
        break;

        case REVERSEINTAKING:
        //motors reverse intake out
        break;

        default:
        //safe
        break;
        
        }
    }


    public void setWantedState(IntakeStates state) {
        this.wantedState = state;
    }    

    public IntakeStates getCurrentState() {
        return this.currentState;
    }

    public IntakeStates getWantedState() {
        return this.wantedState;
    }
}






//67