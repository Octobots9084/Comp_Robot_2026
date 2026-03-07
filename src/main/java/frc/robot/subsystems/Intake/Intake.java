package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Lights.LightAnimations;

// import frc.robot.subsystems.Lights;
public class Intake extends SubsystemBase {

    public IntakeStates currentState = IntakeStates.SAFE;
    public IntakeStates wantedState = IntakeStates.ZERO;
    public IntakeIO io;
    public static Intake instance;
    public boolean alreadyZeroed = false;
    public IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    public int elephantiaissTimer = 0;

    public Intake(IntakeIO io) {
        this.io = io;
        instance = this;

    }

    public static Intake getInstance() {
        return instance;
    }

    @Override
    public void periodic() {
        // This will handle changing between states at the user's request.
        handleStateTransitions();

        // this is where states actually take effect.
        applyStates();
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public void handleStateTransitions() {

        switch (wantedState) {
            case SAFE:
                currentState = IntakeStates.SAFE;
                break;

            case INTAKING:
                // only works if not climbing
                currentState = IntakeStates.INTAKING;
                // Lights.getLightInstance().lightsWantedState = LightAnimations.INTAKING;
                break;

            case EXTENDED:
                // only works if not climbing
                currentState = IntakeStates.EXTENDED;
                break;

            case REVERSEINTAKING:
                // only works if not climbing
                currentState = IntakeStates.REVERSEINTAKING;
                // Lights.getLightInstance().lightsWantedState =
                // LightAnimations.REVERSEINTAKING;

                break;
            case ZERO:
                    currentState = IntakeStates.ZERO;
                break;
            case ELEPHANTIASISPART2:
                    currentState = IntakeStates.ELEPHANTIASISPART2;
                    elephantiaissTimer = 0;
                break;
            default:
                currentState = IntakeStates.SAFE;
                break;

        }
    }

    public void applyStates() {

        switch (currentState) {

            case INTAKING:
            //motors on intake out
            io.setIntakeState(currentState);
            break;

            case EXTENDED:
            //motors off intake out
            io.setIntakeState(currentState);
                
            break;

            case SAFE:
            //motors off intake in
            io.setIntakeState(currentState);
            break;

            case REVERSEINTAKING:
            //motors reverse intake out
            break;
            case ZERO:
                if (io.zeroIntake()) {
                    alreadyZeroed = true;
                    wantedState = IntakeStates.SAFE;
                }else{
                    alreadyZeroed = false;
                }
                break;
            case ELEPHANTIASISPART2:
                if (elephantiaissTimer>0){
                    io.setIntakeState(IntakeStates.INTAKING);
                } else {
                    io.setIntakeState(IntakeStates.PARTIALEXTENTION);
                }
                elephantiaissTimer ++;
                if (elephantiaissTimer > 20){
                    elephantiaissTimer = -20;
                }
                break;
        default:
            io.setIntakeState(currentState);    
            break;
        }
    }

    public void setCurrentState(IntakeStates state) {
        this.currentState = state;
    }

    public IntakeStates getCurrentState() {
        return this.currentState;
    }

    public void setWantedState(IntakeStates state) {
        this.wantedState = state;
    }

    public IntakeStates getWantedState() {
        return this.wantedState;
    }
}
