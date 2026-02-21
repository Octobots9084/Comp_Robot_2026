package frc.robot.subsystems.Shooter.Feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
    public static Feeder currentInstance = null;

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
        io.setFeederVelocity(currentState);
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
