package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;


public class SubsystemWrap<T extends Enum<?>> extends SubsystemBase {
    public T wantedState;
    public T currentState;
    public Logger logger;

    SubsystemWrap() {
        Sys.set(this.getClass(), this);
    }

    @Override
    public void periodic() {
        handleStateTransitions();
        applyStates();
        telemetry();
    }

    /** OVERRIDE THIS */
    public void handleStateTransitions() {};
    /** OVERRIDE THIS */
    public  void applyStates() {};
    /** OVERRIDE THIS */
    public void telemetry() {

    };
        
        public T getCurrentState() {
        return currentState;
    }
    @SuppressWarnings("unchecked")
    public void setCurrentState(Enum<?> state) {
        currentState = (T) state;
    }

    public T getWantedState() {
        return wantedState;
    }

    @SuppressWarnings("unchecked")
    public void setWantedState(Enum<?> state) {
        wantedState = (T) state;
    }

    public boolean wantedStateIs(Enum<?> state) {
        return state == wantedState;
    }
    
    public boolean currentStateIs(Enum<?> state) {
        return state == wantedState;
    }

}