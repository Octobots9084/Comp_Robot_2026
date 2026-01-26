package frc.robot.subsystems.Shooter;

import java.security.spec.ECPublicKeySpec;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter.Feeder.Feeder;
import frc.robot.subsystems.Shooter.Feeder.FeederIO;
import frc.robot.subsystems.Shooter.Feeder.FeederIOInputsAutoLogged;
import frc.robot.subsystems.Shooter.Feeder.FeederStates;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIO;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOInputsAutoLogged;
import frc.robot.subsystems.Shooter.Turret.Turret;
import frc.robot.subsystems.Shooter.Turret.TurretIO;
import frc.robot.subsystems.Shooter.Turret.TurretIOInputsAutoLogged;

public class Shooter extends SubsystemBase{
    ShooterStates currentShooterState;
    ShooterStates wantedShooterState;
    private static Shooter instance = null;
    private final FeederIOInputsAutoLogged feederInputs = new FeederIOInputsAutoLogged();
    private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
    private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();
    public final FeederIO fIO;
    public final FlywheelIO fwIO;
    public final TurretIO tIO;
    public Feeder feeder = new Feeder();
    public Turret turret = new Turret();
    public Flywheel flywheel = new Flywheel();
    public final double prefire = 1;

    private String gameData;

    public Shooter(FeederIO fIO,FlywheelIO fwIO, TurretIO tIO){
        this.fIO = fIO;
        this.fwIO = fwIO;
        this.tIO = tIO;
    }

    public static Shooter setInstance(FeederIO fIO,FlywheelIO fwIO, TurretIO tIO){
        instance = new Shooter(fIO,fwIO,tIO);
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
        // ApplyStates();
        // handleStateTransitions();
        fIO.updateInputs(feederInputs);
        Logger.processInputs("Shooter/Feeder",feederInputs);
        fwIO.updateInputs(flywheelInputs);
        Logger.processInputs("Shooter/Flywheels",flywheelInputs);
        tIO.updateInputs(turretInputs);
        Logger.processInputs("Shooter/Turret and Hood",turretInputs);
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
                if(hub()){
                    wantedShooterState = ShooterStates.BUMP;
                }
                break;
            case BUMP:
                //dont shoot
                break;
            default:
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
                    //if we're on the bump (SHOCKING!!!) ha good one
                    break;
                
                case SAFE:
                    //driver input (presumably)
                    break;

                default:
                    break;
            };
    }
    public boolean hub(){
        if(aim(true) && isHubActive()){
            if(ShooterConstants.driverShoot){
                if(true){//in alliance zone
                    if(true){ // if we have fuel(stop after 2s after no fuel)
                        feeder.setFeederVelocity(FeederStates.SCORING);
                    }else{
                        feeder.setFeederVelocity(FeederStates.OFF);
                    }
                }else{
                    feeder.setFeederVelocity(FeederStates.OFF);
                    return true;
                }

            }
        }
        return false;
    }

    public boolean ferry(){
        if(aim(false)){
            
        }
        return false;
    }

    public boolean aim(boolean atHub){
        return true;
    }


    public boolean isHubActive(){
        double timer = Constants.timer.get();
        gameData = DriverStation.getGameSpecificMessage();
            if(gameData.length() > 0)
            {
                switch (gameData.charAt(0))
                {
                    case 'B' :
                        if(Constants.isBlueAlliance){
                            return (timer <= 10||(timer >= (40-prefire) && timer <= 70 )|| (timer >= (100-prefire) && timer <= 161));
                        }else{
                            return (timer <= 40) || (timer >= (70-prefire) && timer <= 100) || (timer >= (130-prefire) && timer <=  161);
                        }
                    case 'R' :
                        if(!Constants.isBlueAlliance){
                            return (timer <= 10||(timer >= (40-prefire) && timer <= 70 )|| (timer >= (100-prefire) && timer <= 161));
                        }else{
                            return (timer <= 40) || (timer >= (70-prefire) && timer <= 100) || (timer >= (130-prefire) && timer <=  161);
                        }
                        
                    default:
                        return true;
                }
            }else{
                return true;
            }
        }

    
    
}

    
 