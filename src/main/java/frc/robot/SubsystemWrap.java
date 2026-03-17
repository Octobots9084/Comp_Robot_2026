package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsystemWrap<T extends Enum<?>> extends SubsystemBase {
    public T wantedState;
    public T currentState;
    private static List<SubsystemWrap<? extends Enum<?>>> instances;

    public T getCurrentState() {
        return currentState;
    }
    public void setCurrentState(T state) {
        currentState = state;
    }

    public T getWantedState() {
        return wantedState;
    }

    public void setWantedState(T state) {
        wantedState = state;
    }
    
    @Override
    public void periodic() {
        handleStateTransitions();
        applyStates();
        telemetry();
    }

    SubsystemWrap() {
        instances.add(this);
    }

    /** OVERRIDE */
    public void handleStateTransitions() {};
    /** OVERRIDE */
    public  void applyStates() {};
    /** OVERRIDE */
    public void telemetry() {};

}