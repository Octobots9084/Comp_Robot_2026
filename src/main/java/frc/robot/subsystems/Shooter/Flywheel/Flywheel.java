package frc.robot.subsystems.Shooter.Flywheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel;

public class Flywheel extends SubsystemBase {
    private FlywheelStates currentState = FlywheelStates.SAFE;
    public static final double flywheelRadius = 0.1; // in Meters
    public static Flywheel currentFlywheelInstance = null;

    public FlywheelIO io = new FlywheelIOTalonFX();

    public Flywheel() {
        currentFlywheelInstance = this;
    }

    public static Flywheel getInstance() {
        return currentFlywheelInstance;
    }

    public static void setInstance(Flywheel instance) {
        currentFlywheelInstance = instance;
    }

    public FlywheelStates getCurrentState() {
        return this.currentState;
    }

    public void setFlywheelVelocity(FlywheelStates currentState) {
        this.currentState = currentState;
        io.setFlywheelVelocity(currentState);
    }

    public void setFlywheelVelocity(double rps) {
        io.setFlywheelVelocity(rps);
    }

    public double getRightMotorVelocity() {
        return io.getRightMotorVelocity();
    }

    public boolean FlywheelInTolerance(double tolerance) {
        boolean inTolerance = io.FlywheelInTolerance(tolerance);
        Logger.recordOutput("flywheel in tolerance", inTolerance);
        return inTolerance;
    }
}
