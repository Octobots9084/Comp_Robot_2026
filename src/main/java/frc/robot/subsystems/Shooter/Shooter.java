package frc.robot.subsystems.Shooter;

import java.security.spec.ECPublicKeySpec;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ButtonConfig;
import frc.robot.Constants;
import frc.robot.Constants;
import frc.robot.subsystems.Lights.LightAnimations;
import frc.robot.subsystems.Lights.Lights;
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
import frc.robot.subsystems.Drive.SwerveSubsystem;

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
    public CANrange lemonDetector = new CANrange(Constants.lemonDetector,Constants.krakenBus);
    public static boolean driverOverride = false;
    private String gameData;
    private SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    private double lemonDetectionTimestamp;

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
        SmartDashboard.putBoolean("HubAcivity",isHubActive());
    }
     
    public void ApplyStates(){
        switch(currentShooterState){
            case SAFE:
                //stop the flywheel
                break;
            case FERRY:
                if(ferry()){
                    wantedShooterState = ShooterStates.BUMP;
                    Lights.getLightInstance().lightsWantedState = LightAnimations.CANTSHOOT;
                }
                break;
            case HUB:
                if(hub()){
                    wantedShooterState = ShooterStates.BUMP;
                    Lights.getLightInstance().lightsWantedState = LightAnimations.CANTSHOOT;
                }
                break;
            case BUMP:
                //dont shoot
                if(swerve.onRamp(0,3)){//!tilted
                    if(true){ //in alliance zone
                        wantedShooterState = ShooterStates.HUB;
                        Lights.getLightInstance().lightsWantedState = LightAnimations.SHOOTREADYCONTINIOUS;
                    }else{
                        wantedShooterState = ShooterStates.FERRY;
                        Lights.getLightInstance().lightsWantedState = LightAnimations.SHOOTREADYCONTINIOUS;

                    }
                }
                break;
            default:
                break;
        }

    }

 public void handleStateTransitions(){
        switch (wantedShooterState) {
                case HUB:
                    //if we're on our side of the field
                    if(true){//!tilted and in alliance
                        currentShooterState = ShooterStates.HUB;
                    }
                    break;
                
                case FERRY:
                    //if we're in neutral or enemy zone
                    if(true){//!tilted and !in alliance
                        currentShooterState = ShooterStates.FERRY;
                    }
                    break;
                
                case BUMP:
                    //if we're on the bump (SHOCKING!!!) 
                    if(true){ //robot is tilted
                        currentShooterState = ShooterStates.BUMP;
                    }
                    break;
                
                case SAFE:
                    //driver input (presumably)
                    break;

                default:
                    break;
            };
    }
    //automatically shoots a ball if it can score and allows zeo to override some factors
    public boolean hub(){
        if(aim(true) && isHubActive()){
            if(driverOverride){
                feeder.setFeederVelocity(FeederStates.SCORING);
                // manual shooting
                Lights.getLightInstance().lightsWantedState = LightAnimations.SHOOTREADYMANUAL;
            }else{
                if(true && swerve.onRamp(0, 3)){//in alliance zone
                    if(hasFuel()){ // if we have fuel(stop after 2s after no fuel)
                        feeder.setFeederVelocity(FeederStates.SCORING);
                        //automatic shooting
                        Lights.getLightInstance().lightsWantedState = LightAnimations.SHOOTREADYCONTINIOUS;
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

    public boolean inAllianceZone(){
        return true; //TODO rui needs to make working pose...
    }

    public boolean hasFuel(){
        if(lemonDetector.getDistance().getValueAsDouble()<7){
            lemonDetectionTimestamp  = Constants.timer.get();
        }
        if(Constants.timer.get() - lemonDetectionTimestamp >= 1.5){
            return false;
        }
        return true;
    }

    public boolean ferry(){
        if(aim(false)){
            if(driverOverride){
                feeder.setFeederVelocity(FeederStates.FERRYING);
            }else{
                if(inAllianceZone()){//!in alliance zone
                    if(hasFuel()){ // if we have fuel(stoap after 2s after no fuel)
                        feeder.setFeederVelocity(FeederStates.FERRYING);
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

    
 