package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    ShooterStates currentShooterState;
    private static Shooter instance = null;
    public final ShooterIO io;
    public Shooter(ShooterIO io){
        this.io = io;
    }

    public static Shooter setInstance(ShooterIO io){
        instance = new Shooter(io);
        return instance;
    }
    public static Shooter getInstance(){
        if (instance == null){
            throw new IllegalStateException("Shooter Instance Not Set");

        }
        return instance;
    }
    
    @Override
    public void periodic(){
        ApplyStates();
        handleStateTransitions();

    }
    public void ApplyStates(){
        switch(currentShooterState){
            case SAFE:
                //stop the flywheel
                break;
            case FERRY:
                //shoot over bump
                break;
            case HUB:
                //shoot at our hub
                break;
            case BUMP:
                //dont shoot
            break;
        }
    }

 public void handleStateTransitions(){
        switch (currentShooterState) {
                case HUB:
                    //if we're on our side of the field
                    break;
                
                case FERRY:
                    //if we're in neutral or enemy zone
                    break;
                
                case BUMP:
                    //if we're on the bump (SHOCKING!!!)
                    break;
                
                case SAFE:
                    //driver input (presumably)
                    break;
            };
    }
}

    
 