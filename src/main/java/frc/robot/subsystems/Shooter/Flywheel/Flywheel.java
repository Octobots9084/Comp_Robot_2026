package frc.robot.subsystems.Shooter.Flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel;

public class Flywheel extends SubsystemBase{
     public static Flywheel currentFlywheelInstance = null;

    public FlywheelIO io = new FlywheelIOTalonFX();
    
    public Flywheel(){
        currentFlywheelInstance = this;
    }

    public static Flywheel getInstance(){
        return currentFlywheelInstance;
    }

    public static void setInstance(Flywheel instance){
        currentFlywheelInstance = instance;
    }

    public void setFlywheelVelocity(FlywheelStates currentState){
        io.setFlywheelVelocity(currentState);
    }

    public double[] getFlywheelVelocity(){
        return io.getFlywheelVelocity();
    }

    public double getLeftMotorVelocity(){
        return io.getLeftMotorVelocity();
    }

    public double getRightMotorVelocity(){
        return io.getRightMotorVelocity();
    }

    public boolean FlywheelInTolerance(double tolerance){
        return io.FlywheelInTolerance(tolerance);
    }
}
