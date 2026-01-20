package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    public static IntakeIO io = new IntakeIOTalonFX();
    public static Intake currentInstance;
    
    IntakeStates currentState = IntakeStates.SAFE;
    IntakeStates wantedState = IntakeStates.SAFE;

    public Intake(){
        currentInstance = this;
    }

    public static Intake getInstance(){
        return currentInstance;
    }

    public static void setInstance(Intake instance){
        currentInstance = instance;
    }
    
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

    public void setIntakePosition(IntakeStates state){
        io.setIntakeState(state);
    }

    public double getIntakeVelocity(){
        return io.getIntakeVelocity();
    }

    public double getPivotPosition(){
        return io.getPivotPosition();
    }

    public boolean pivotInTolerance(double tolerance){
        return io.pivotInTolerance(tolerance);
    }

    public boolean rollerInTolerance(double tolerance){
        return io.rollerInTolerance(tolerance);
    }

    public boolean intakeInTolerance(double tolerance){
        return io.intakeInTolerance(tolerance);
    }
}