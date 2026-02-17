package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.ShooterAngle;
import frc.robot.subsystems.Vision.ShooterAngleCalculator;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Lights.LightAnimations;
import frc.robot.subsystems.Lights.Lights;
import frc.robot.subsystems.Shooter.Feeder.Feeder;
import frc.robot.subsystems.Shooter.Feeder.FeederIO;
import frc.robot.subsystems.Shooter.Feeder.FeederIOInputsAutoLogged;
import frc.robot.subsystems.Shooter.Feeder.FeederStates;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIO;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOInputsAutoLogged;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelStates;
import frc.robot.subsystems.Shooter.Turret.Turret;
import frc.robot.subsystems.Shooter.Turret.TurretIO;
import frc.robot.subsystems.Shooter.Turret.TurretIOInputsAutoLogged;

public class Shooter extends SubsystemBase{
    private ShooterAngle pastShooterAngle = new ShooterAngle(0, 0);
    public ShooterStates currentShooterState = ShooterStates.HUB;//SAFE; //should be safe but useing hub for testing
    public ShooterStates wantedShooterState = ShooterStates.HUB;
    private static Shooter instance = null;
    private final FeederIOInputsAutoLogged feederInputs = new FeederIOInputsAutoLogged();
    private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
    private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();
    private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
    public final FeederIO fIO;
    public final FlywheelIO fwIO;
    public final TurretIO tIO;
    public final ShooterIO sIO;
    public SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    public final CommandXboxController coDriverController;
    public Feeder feeder = new Feeder();
    public Turret turret;
    public Flywheel flywheel = new Flywheel();
    public final double prefire = 1;
    public CANrange lemonDetector = new CANrange(Constants.lemonDetector,Constants.krakenBus);
    public static boolean driverOverride = false;
    private String gameData;
    private double lemonDetectionTimestamp;
    public double turretAim = -0.1;
    private ShooterAngle shooterAngle;
    private Pose2d hubPose = new Pose2d(10, 10, new Rotation2d());

    public Shooter(FeederIO fIO,FlywheelIO fwIO, TurretIO tIO, ShooterIO sIO, CommandXboxController coDriverController){
        this.fIO = fIO;
        this.fwIO = fwIO;
        this.tIO = tIO;
        this.sIO = sIO;
        this.coDriverController = coDriverController;
        instance = this;
        turret  = new Turret(tIO);
    }

    public static Shooter setInstance(FeederIO fIO,FlywheelIO fwIO, TurretIO tIO,ShooterIO sIO, CommandXboxController coDriverController){
        instance = new Shooter(fIO,fwIO,tIO,sIO,coDriverController);
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
        fIO.updateInputs(feederInputs);
        Logger.processInputs("Shooter/Feeder",feederInputs);
        fwIO.updateInputs(flywheelInputs);
        Logger.processInputs("Shooter/Flywheels",flywheelInputs);
        tIO.updateInputs(turretInputs);
        Logger.processInputs("Shooter/Turret and Hood",turretInputs);
        sIO.updateInputs(shooterInputs);
        Logger.processInputs("Shooter/Shooter",shooterInputs);
        SmartDashboard.putBoolean("HubAcivity",isHubActive());
    }
     
    public void ApplyStates(){
        switch(currentShooterState){
            case SAFE:
                //stop the flywheel
                // Turret.getInstance().setTurretPosition(0);
                feeder.setFeederVelocity(FeederStates.OFF);
                flywheel.setFlywheelVelocity(FlywheelStates.SAFE);
                turret.setHoodPosition(0);
                turret.setTurretPosition(0);
                break;
            case MANUAL:
            //joystick controlls turret
            tIO.setTurretPosition(getTurretPosFromJoystick()); 
            tIO.setHoodPosition(getHoodPosFromJoystick());
            case FERRY:
                // if(ferry()){
                //     wantedShooterState = ShooterStates.BUMP;
                       // Lights.getLightInstance().lightsWantedState = LightAnimations.CANTSHOOT;
                // }
                break;
            case HUB:
                shooterAngle = ShooterAngleCalculator.getShooterAngleToHub(
                    swerve.io.getChassisSpeeds().vxMetersPerSecond,
                    swerve.io.getChassisSpeeds().vxMetersPerSecond,
                    hubPose.getX() - swerve.io.getPose2d().getY(),
                    hubPose.getY() - swerve.io.getPose2d().getX(),
                    20// (Flywheel.getInstance().getFlywheelVelocity()[1] * Flywheel.flywheelRadius + Flywheel.getInstance().getFlywheelVelocity()[0] * Flywheel.flywheelRadius)/2.0
                );
                if (shooterAngle != null){
                    pastShooterAngle = shooterAngle;
                }
                SmartDashboard.putNumber("shooterHoodAngle",pastShooterAngle.hoodRotation);
                SmartDashboard.putNumber("shooterAngle",pastShooterAngle.turretRotation);
                
                // if(hub()){
                //     wantedShooterState = ShooterStates.BUMP;
                    // Lights.getLightInstance().lightsWantedState = LightAnimations.CANTSHOOT;
                // }
                // 
                
            double gyro = SwerveSubsystem.getInstance().io.getGyro();

            // Wrap properly
            gyro = ((gyro % 360) + 360) % 360;

            // Invert gyro direction BEFORE scaling
            gyro = 360 - gyro;

            double turretAngle = -(gyro / 360.0) + 0.5;

            turret.setTurretPosition(turretAngle);

            feeder.setFeederVelocity(FeederStates.SPITTING);
            flywheel.setFlywheelVelocity(FlywheelStates.SPIT);
            turret.setHoodPosition(turret.spitTurrentHood);
                break;
            case BUMP:
                //dont shoot
                if(swerve.onRamp(0,3)){//!tilted
                    if(true){ //in alliance zone
                        wantedShooterState = ShooterStates.HUB;
                        // Lights.getLightInstance().lightsWantedState = LightAnimations.SHOOTREADYCONTINIOUS;
                    }else{
                        wantedShooterState = ShooterStates.FERRY;
                        // Lights.getLightInstance().lightsWantedState = LightAnimations.SHOOTREADYCONTINIOUS;

                    }
                }
                break;
            case SPIT:
                feeder.setFeederVelocity(FeederStates.SPITTING);
                flywheel.setFlywheelVelocity(FlywheelStates.SPIT);
                turret.setHoodPosition(turret.spitTurrentHood);
                turret.setTurretPosition(turretAim);
                break;
            case ZERO:

            if(turret.io.turretZeroed()){
                    currentShooterState = ShooterStates.SAFE;
                }
                break;
            default:
                break;
        }

    }


    //state transitions for spit and manual needed
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
                    currentShooterState = ShooterStates.SAFE;
                    break;
                case ZERO:
                    currentShooterState = ShooterStates.ZERO;
                    break;
                case MANUAL:
                    currentShooterState = ShooterStates.MANUAL;
                    break;
                case SPIT:
                    currentShooterState = ShooterStates.SPIT;
                default:
                    break;
            };
    }
    public boolean Shootable(){
        if(!swerve.onRamp(1,3) && ((Shooter.getInstance().inAllianceZone() && isHubActive()) || (!Shooter.getInstance().inAllianceZone()))){
            return true;
        }else{
            return false;
        }

    }
    //automatically shoots a ball if it can score and allows zeo to override some factors
    public boolean hub(){
        // there is no feederrequest so this causes an error
        if(aim(true) && isHubActive()){
            if(driverOverride){
                feeder.setFeederVelocity(FeederStates.SCORING);
                // manual shooting
                // Lights.getLightInstance().lightsWantedState = LightAnimations.SHOOTREADYMANUAL;
            }else{
                if(true && swerve.onRamp(0, 3)){//in alliance zone
                    if(hasFuel()){ // if we have fuel(stop after 2s after no fuel)
                        feeder.setFeederVelocity(FeederStates.SCORING);
                        //automatic shooting
                        // Lights.getLightInstance().lightsWantedState = LightAnimations.SHOOTREADYCONTINIOUS;
                    }else{
                        // feeder.setFeederVelocity(FeederStates.OFF);
                    }
                }else{
                    // feeder.setFeederVelocity(FeederStates.OFF);
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
                if(!inAllianceZone()){//!in alliance zone
                    if(hasFuel()){ // if we have fuel(stoap after 2s after no fuel)
                        feeder.setFeederVelocity(FeederStates.FERRYING);
                    }else{
                        // feeder.setFeederVelocity(FeederStates.OFF);
                    }
                }else{
                    // feeder.setFeederVelocity(FeederStates.OFF);
                    return true;
                }
            }
        }
        return false;
    }

    public boolean aim(boolean atHub){
        return true;
    }

    public double getTurretPosFromJoystick(){
        return tIO.getTurretPosition() + 0.05 * MathUtil.applyDeadband(coDriverController.getLeftX(), Constants.leftYDeadband);
    }

    public double getHoodPosFromJoystick(){
        return tIO.getHoodPosition() + 0.05 * -MathUtil.applyDeadband(coDriverController.getLeftY(), Constants.leftXDeadband);
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

    
 