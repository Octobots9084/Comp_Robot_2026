package frc.robot.subsystems.Shooter.Flywheel;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase{
    public static final double flywheelRadius = 0.1; // in M
    public static Flywheel currentInstance = null;

    public FlywheelIO io = new FlywheelIOTalonFX();
    
    public Flywheel(){
        currentInstance = this;
    }

    public static Flywheel getInstance(){
        return currentInstance;
    }

    public static void setInstance(Flywheel instance){
        currentInstance = instance;
    }

    public void setFlyWheelVelocity(FlywheelStates currentState){
        io.setFlyWheelVelocity(currentState.flywheelRPS);
    }

    public AngularVelocity getFlyWheelVelocity(){
        return io.getFlyWheelVelocity();
    }

    public boolean flywheelInTolerance(double tolerance){
        return io.flywheelInTolerance(tolerance);
    }

    public void setTopRollerVelocity(FlywheelStates currentState){
        io.setTopRollerVelocity(currentState.topRollerRPS);
    }

    public void setAllFlywheelVelocities(FlywheelStates currentState){
        setTopRollerVelocity(currentState);
        setFlyWheelVelocity(currentState);
    }

    public AngularVelocity getTopRollerVelocity(){
        return io.getTopRollerVelocity();
    }

    public boolean topRollerInTolerance(double tolerance){
        return io.topRollerInTolerance(tolerance);
    }
}
