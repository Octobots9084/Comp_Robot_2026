package frc.robot.subsystems.Shooter.Feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
    public static Feeder currentInstance = null;
    private FeederStates currentState = FeederStates.OFF;

    public FeederIO io = new FeederIOTalonFX();

    public Feeder() {
        currentInstance = this;
    }

    public static Feeder getInstance() {
        return currentInstance;
    }

    public static void setInstance(Feeder instance) {
        currentInstance = instance;
    }

    public void setFeederVelocity(FeederStates currentState) {
        if (this.currentState != currentState && currentState == FeederStates.SCORING)
            io.resetUpToSpeed();
        this.currentState = currentState;
        io.setFeederVelocity(currentState);
    }

    public FeederStates getCurrentState() {
        return this.currentState;
    }

    public double[] getFeederVelocity() {
        return io.getFeederVelocity();
    }

    public double getSingleFeederVelocity() {
        return io.getVerticalFeederVelocity();
    }

    public double getMultiFeederVelocity() {
        return io.getSpindexerVelocity();
    }

    public boolean feederInTolerance(double tolerance) {
        return io.feederInTolerance(tolerance);
    }
}
