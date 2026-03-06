package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.ShooterAngle;
import frc.robot.subsystems.Vision.ShooterAngleCalculator;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Intake.IntakeStates;
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

public class Shooter extends SubsystemBase {
    private ShooterAngle pastShooterAngle = new ShooterAngle(0, 0);
    public ShooterStates currentShooterState = ShooterStates.HUB;// SAFE; //should be safe but useing hub for testing
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
    // public final CommandXboxController coDriverController;
    public Feeder feeder = new Feeder();
    public Turret turret;
    public boolean alreadyZeroed = false;
    public Flywheel flywheel = new Flywheel();
    public final double prefire = 1;
    public static boolean driverOverride = false;
    private String gameData;
    private double lemonDetectionTimestamp;
    public double turretAim = -0.1;
    private ShooterAngle shooterAngle;
    private Translation2d hubPoseBlue = new Translation2d(4.6228, 4.02082);
    private Translation2d hubPoseRed = new Translation2d(11.88974, 4.02082);
    private Translation2d blueFerryOutpost = new Translation2d(4.6239 - 2,2.011);
    private Translation2d redFerryOutpost = new Translation2d(11.917 + 2,6.031);
    private Translation2d blueFerryDepot = new Translation2d(4.6239 - 2,6.03);
    private Translation2d redFerryDepot = new Translation2d(11.917 + 2,2.011);
    public boolean isAimedAtHub;
    public double shooterCalculatorVelocity;

    public Shooter(FeederIO fIO, FlywheelIO fwIO, TurretIO tIO, ShooterIO sIO) {
        this.fIO = fIO;
        this.fwIO = fwIO;
        this.tIO = tIO;
        this.sIO = sIO;
        instance = this;
        turret = new Turret(tIO);
    }

    public static Shooter setInstance(FeederIO fIO, FlywheelIO fwIO, TurretIO tIO, ShooterIO sIO) {
        instance = new Shooter(fIO, fwIO, tIO, sIO);
        return instance;
    }

    public static Shooter getInstance() {

        if (instance == null) {
            throw new IllegalStateException("Shooter Instance Not Set");
        }
        return instance;
    }

    @Override
    public void periodic() {
        ApplyStates();
        handleStateTransitions();
        fIO.updateInputs(feederInputs);
        Logger.processInputs("Shooter/Feeder", feederInputs);
        fwIO.updateInputs(flywheelInputs);
        Logger.processInputs("Shooter/Flywheels", flywheelInputs);
        tIO.updateInputs(turretInputs);
        Logger.processInputs("Shooter/Turret and Hood", turretInputs);
        sIO.updateInputs(shooterInputs);
        Logger.processInputs("Shooter/Shooter", shooterInputs);
        // SmartDashboard.putBoolean("HubAcivity", isHubActive());
    }

    public void ApplyStates() {
        switch (currentShooterState) {
            case SAFE:
                // stop the flywheel
                feeder.setFeederVelocity(FeederStates.OFF);
                flywheel.setFlywheelVelocity(FlywheelStates.SAFE);
                break;
            case MANUAL:
                // joystick controlls turret
                // tIO.setTurretPosition(getTurretPosFromJoystick());
                // tIO.setHoodPosition(getHoodPosFromJoystick());
            case FERRY:
                isAimedAtHub = aimFerry();
                if(!swerve.isInAllianceZone() && !swerve.onRamp(0,5)){
                    if(driverOverride){
                        flywheel.setFlywheelVelocity(FlywheelStates.HUB);
                        if(isAimedAtHub && flywheel.FlywheelInTolerance(18)){
                            feeder.setFeederVelocity(FeederStates.SCORING);
                        }else{
                            feeder.setFeederVelocity(FeederStates.OFF);
                        }
                    }else{
                        feeder.setFeederVelocity(FeederStates.OFF);
                        flywheel.setFlywheelVelocity(FlywheelStates.SAFE);
                    }
                }
                else {
                    wantedShooterState = ShooterStates.BUMP;
                }
                break;
            case HUB:
                isAimedAtHub = isAimedAtHub();
                if(swerve.isInAllianceZone()){
                    if(driverOverride){
                        flywheel.setFlywheelVelocity(37+10*((getDistanceToHub()-1.237)/(5.476-1.237)));
                        // flywheel.setFlywheelVelocity(FlywheelStates.HUB);

                        if(isAimedAtHub && flywheel.FlywheelInTolerance(8)){
                            feeder.setFeederVelocity(FeederStates.SCORING);
                        }else{
                            feeder.setFeederVelocity(FeederStates.OFF);
                        }
                    }else{
                        feeder.setFeederVelocity(FeederStates.OFF);
                        flywheel.setFlywheelVelocity(FlywheelStates.SAFE);
                    }
                }else{
                    wantedShooterState = ShooterStates.BUMP;
                }
                break;
            case AUTOFERRY:
                isAimedAtHub = aimFerry();
                if(!swerve.isInAllianceZone() && !swerve.onRamp(0,5)){
                        flywheel.setFlywheelVelocity(FlywheelStates.HUB);
                        if(isAimedAtHub && flywheel.FlywheelInTolerance(18)){
                            feeder.setFeederVelocity(FeederStates.SCORING);
                        }else{
                            feeder.setFeederVelocity(FeederStates.OFF);
                        } 
                }
                else {
                    wantedShooterState = ShooterStates.BUMP;
                }
                break;
            case AUTOHUB:
               isAimedAtHub = isAimedAtHub();
                flywheel.setFlywheelVelocity(37+10*((getDistanceToHub()-1.237)/(5.476-1.237)));
                // flywheel.setFlywheelVelocity(FlywheelStates.HUB);
                
                if(isAimedAtHub && flywheel.FlywheelInTolerance(8)){
                    feeder.setFeederVelocity(FeederStates.SCORING);
                }else{
                    feeder.setFeederVelocity(FeederStates.OFF);
                }
                break;
            case BUMP:
                if (!swerve.onRamp(0, 5)) {
                    if (swerve.isInAllianceZone()) {
                        if (DriverStation.isAutonomousEnabled()){
                        wantedShooterState = ShooterStates.AUTOHUB;
                        } else{
                        wantedShooterState = ShooterStates.HUB;
                        }
                    } else {
                        if (DriverStation.isAutonomousEnabled()){
                        wantedShooterState = ShooterStates.AUTOFERRY;
                        } else{
                        wantedShooterState = ShooterStates.FERRY;
                        }
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
            if (turret.io.turretZeroed()) {
                    wantedShooterState = ShooterStates.SAFE;
                    alreadyZeroed = true;
                }
                break;
            default:
                break;
        }

    }

    // state transitions for spit and manual needed
    public void handleStateTransitions() {
        switch (wantedShooterState) {
            case HUB:
                // if we're on our side of the field
                if (true) {// !tilted and in alliance
                    currentShooterState = ShooterStates.HUB;
                }
                break;
            case AUTOHUB:
                // if we're on our side of the field
                if (true) {// !tilted and in alliance
                    currentShooterState = ShooterStates.AUTOHUB;
                }
                break;

            case FERRY:
                // if we're in neutral or enemy zone
                if (true) {// !tilted and !in alliance
                    currentShooterState = ShooterStates.FERRY;
                }
                break;

            case BUMP:
                // if we're on the bump (SHOCKING!!!)
                if (true) { // robot is tilted
                    currentShooterState = ShooterStates.BUMP;
                }
                break;

            case SAFE:
                // driver input (presumably)
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
        }
        ;
    }

    public boolean Shootable() {
        if (!swerve.onRamp(1, 0.3) && ((swerve.isInAllianceZone() && isHubActive())
                || (!swerve.isInAllianceZone()))) {
            return true;
        } else {
            return false;
        }

    }

    // public boolean hubManualShoot() {
    //     if (aimHub() /* && isHubActive() */) {
    //         // TODO ACTIVELY NEEDS TO BE FIXED
    //         if (driverOverride) {
    //             flywheel.setFlywheelVelocity(FlywheelStates.HUB);
    //             // if(flywheel.FlywheelInTolerance(0.5)){
    //             feeder.setFeederVelocity(FeederStates.SCORING);
    //             // }
    //         } else {
    //             feeder.setFeederVelocity(FeederStates.OFF);
    //         }
    //     }
    //     return false;
    // }

    public boolean shootHub() {
        if (isAimedAtHub() /*&& isHubActive()*/) {
            // TODO ACTIVELY NEEDS TO BE FIXED
            if(swerve.isInAllianceZone() && !swerve.onRamp(0, 0.3)){
                // if (driverOverride) {
                    flywheel.setFlywheelVelocity(FlywheelStates.HUB);
                    if (flywheel.FlywheelInTolerance(12)) {
                        feeder.setFeederVelocity(FeederStates.SCORING);
                    }
                // } else {
                //     feeder.setFeederVelocity(FeederStates.AGITATION);
                // }
            }else{
                return true;
            }
        }
        return false;
    }

    public double getDistanceToHub(){
        double XToHub;
        double YToHub;
        if(Constants.isBlueAlliance){
            YToHub = hubPoseBlue.getY() - swerve.io.getPose2d().getY()
                - Constants.TurretDistFromCenter
                    * Math.sin((Math.PI * (swerve.io.getGyro() / 180)) + (Math.PI * 5) / 4);
            XToHub =  hubPoseBlue.getX() - swerve.io.getPose2d().getX()
                - Constants.TurretDistFromCenter
                    * Math.cos((Math.PI * (swerve.io.getGyro() / 180)) + (Math.PI * 5) / 4);
        } else {
            YToHub = hubPoseRed.getY() - swerve.io.getPose2d().getY()
                - Constants.TurretDistFromCenter
                    * Math.sin((Math.PI * (swerve.io.getGyro() / 180)) + (Math.PI * 5) / 4);
            XToHub =  hubPoseRed.getX() - swerve.io.getPose2d().getX()
                - Constants.TurretDistFromCenter
                    * Math.cos((Math.PI * (swerve.io.getGyro() / 180)) + (Math.PI * 5) / 4);
        }
        
            
        // SmartDashboard.putNumber("X distance to hub",XToHub);
        // SmartDashboard.putNumber("Y distance to hub",YToHub);
        return Math.sqrt(YToHub*YToHub+XToHub*XToHub);
    }

    public boolean isAimedAtHub() {
        if (Constants.isBlueAlliance) {
        double YToHub = hubPoseBlue.getY() - swerve.io.getPose2d().getY()
            - Constants.TurretDistFromCenter
                * Math.sin((Math.PI * (swerve.io.getGyro() / 180)) + (Math.PI * 5) / 4);
        double XToHub =  hubPoseBlue.getX() - swerve.io.getPose2d().getX()
            - Constants.TurretDistFromCenter
                * Math.cos((Math.PI * (swerve.io.getGyro() / 180)) + (Math.PI * 5) / 4);
            
            double toHub = Math.sqrt(YToHub*YToHub+XToHub*XToHub);
            
            // SmartDashboard.putNumber("distance to hub",toHub);

            

            shooterAngle = ShooterAngleCalculator.getShooterAngleToHub(
                    swerve.io.getChassisSpeeds().vxMetersPerSecond,
                    swerve.io.getChassisSpeeds().vyMetersPerSecond,
                    XToHub,
                    YToHub,
                    7.098+1.34*((getDistanceToHub()-1.237)/(5.476-1.237))// 0.5 * Constants.FlywheelDiamiter * Math.PI *
                       // (Flywheel.getInstance().getFlywheelVelocity()[1]*2*Math.PI *
                       // Flywheel.flywheelRadius +
                       // Flywheel.getInstance().getFlywheelVelocity()[0]*2*Math.PI *
                       // Flywheel.flywheelRadius)/2.0
            );

            // SmartDashboard.putNumber("flywheel modulated speed",  6.22273+0.84091*((getDistanceToHub()-1.237)/(5.476-1.237)));

        } else {

            double YToHub = hubPoseRed.getY() - swerve.io.getPose2d().getY()
                - Constants.TurretDistFromCenter
                    * Math.sin((swerve.io.getPose2d().getRotation().getRadians()) + (Math.PI * 5) / 4);
            double XToHub =  hubPoseRed.getX() - swerve.io.getPose2d().getX()
                - Constants.TurretDistFromCenter
                    * Math.cos((swerve.io.getPose2d().getRotation().getRadians()) + (Math.PI * 5) / 4);
            
            double xTurretOffset = Constants.TurretDistFromCenter
                    * Math.cos((swerve.io.getPose2d().getRotation().getRadians()) + (Math.PI * 5) / 4);

            double yTurretOffset = Constants.TurretDistFromCenter
                    * Math.sin((swerve.io.getPose2d().getRotation().getRadians()) + (Math.PI * 5) / 4);
                    
            double toHub = Math.sqrt(YToHub*YToHub+XToHub*XToHub);

            shooterCalculatorVelocity = 7.09786+1.91*((getDistanceToHub()-1.237)/(5.476-1.237));
            shooterAngle = ShooterAngleCalculator.getShooterAngleToHub(
                    swerve.io.getChassisSpeeds().vxMetersPerSecond,
                    swerve.io.getChassisSpeeds().vyMetersPerSecond,
                    XToHub,
                    YToHub,
                    shooterCalculatorVelocity);
            // SmartDashboard.putNumber("flywheel modulated speed",  6.22273+0.84091*((getDistanceToHub()-1.237)/(5.476-1.237)));
        }

        if (shooterAngle != null) {
            pastShooterAngle = shooterAngle;
        }
        // SmartDashboard.putNumber("shooterHoodAngle", pastShooterAngle.hoodRotation);
        // SmartDashboard.putNumber("shooterAngle", pastShooterAngle.turretRotation);

        double rotation = SwerveSubsystem.getInstance().getRobotPose().getRotation().getRadians();

        double proposedAngle = (pastShooterAngle.turretRotation - rotation) + Math.PI % (2*Math.PI) - Math.PI;

        if (
            (proposedAngle - 2*Math.PI) > Constants.minTurretAngle
            &&
            Math.abs((rotation - 2*Math.PI)-pastShooterAngle.turretRotation) < Math.abs((rotation)-pastShooterAngle.turretRotation)
            )
        {
            proposedAngle = proposedAngle - 2*Math.PI;
        }
        else if (
            (proposedAngle + 2*Math.PI) < Constants.maxTurretAngle 
            && 
            Math.abs((rotation + 2*Math.PI) - pastShooterAngle.turretRotation) < Math.abs((rotation)-pastShooterAngle.turretRotation))
        {
            proposedAngle = proposedAngle + 2*Math.PI;
        }

        // turret.setTurretPosition(proposedAngle/(2*Math.PI));
        Logger.recordOutput("CalculatedTurretAngle", ((pastShooterAngle.turretRotation - rotation) + Math.PI % (2*Math.PI) - Math.PI)/(2*Math.PI));
        Logger.recordOutput("CalculatedCorrectedTurretAngle", proposedAngle/(2*Math.PI));


        // double rotation = -180;
        // rotation = rotation % 360;
        // rotation = 360 - rotation;

        // // Invert gyro direction BEFORE scaling
        // // double offset = 190;//pastShooterAngle.turretRotation * (180.0 / Math.PI);
        // double offset = (pastShooterAngle.turretRotation * (180.0 / Math.PI));// + alphabotJankboticsOffset;
        // rotation += offset + 255; //maybe 255
        // rotation = rotation % 360;
        // double turretAngle = -(rotation / 360.0);

        // turret.setTurretPosition(turretAngle);
        // double hoodInverted = 85 - (pastShooterAngle.hoodRotation * 180) / Math.PI;
        // // double hoodInverted = 85-(67);
        // double hoodRelative = hoodInverted / 360;


        turret.setHoodPosition(pastShooterAngle.hoodRotation/(2.0*Math.PI));
        Logger.recordOutput("CalculatedHoodAngle", pastShooterAngle.hoodRotation/(2*Math.PI));


        if (turret.hoodInTolerance(.05) && turret.turretInTolerance(0.05) && turret.io.getAimedToShoot()) {
            return true;
        }
        return false;
    }



    public boolean aimFerry() {
        Translation2d target;
        if(!Constants.isBlueAlliance){
            double redFerryDepotDistance = Math.sqrt(Math.pow(redFerryDepot.getX() - swerve.io.getPose2d().getX(),2)+Math.pow((redFerryDepot.getY() - swerve.io.getPose2d().getY()),2));
            double redFerryOutpostDistance = Math.sqrt(Math.pow(redFerryOutpost.getX() - swerve.io.getPose2d().getX(),2)+Math.pow((redFerryOutpost.getY() - swerve.io.getPose2d().getY()),2));
            if(redFerryDepotDistance <= redFerryOutpostDistance){
                target = redFerryDepot;
            }else{
                target = redFerryOutpost;
            }
        }else{
            double blueFerryDepotDistance = Math.sqrt(Math.pow(blueFerryDepot.getX() - swerve.io.getPose2d().getX(),2)+Math.pow((blueFerryDepot.getY() - swerve.io.getPose2d().getY()),2));
            double blueFerryOutpostDistance = Math.sqrt(Math.pow(blueFerryOutpost.getX() - swerve.io.getPose2d().getX(),2)+Math.pow((blueFerryOutpost.getY() - swerve.io.getPose2d().getY()),2));
            if(blueFerryDepotDistance <= blueFerryOutpostDistance){
                target = blueFerryDepot;
            }else{
                target = blueFerryOutpost;
            }
        }
        shooterAngle = ShooterAngleCalculator.getShooterAngleToFerry(
                    swerve.io.getChassisSpeeds().vxMetersPerSecond,
                    swerve.io.getChassisSpeeds().vxMetersPerSecond,
                    target.getX() - swerve.io.getPose2d().getX()
                            + Constants.TurretDistFromCenter
                                    * Math.cos((Math.PI * (swerve.io.getGyro() / 180)) + (Math.PI * 3) / 4),
                    target.getY() - swerve.io.getPose2d().getY()
                            + Constants.TurretDistFromCenter
                                    * Math.sin((Math.PI * (swerve.io.getGyro() / 180)) + (Math.PI * 3) / 4),
                    7.77);

        if (shooterAngle != null) {
            pastShooterAngle = shooterAngle;
        }
        // SmartDashboard.putNumber("shooterHoodAngle", pastShooterAngle.hoodRotation);
        // SmartDashboard.putNumber("shooterAngle", pastShooterAngle.turretRotation);

        double rotation = SwerveSubsystem.getInstance().getRobotPose().getRotation().getDegrees();
        rotation = rotation % 360;
        rotation = 360 - rotation;

        // Invert gyro direction BEFORE scaling
        // double offset = 190;//pastShooterAngle.turretRotation * (180.0 / Math.PI);
        double alphabotJankboticsOffset = 0;
        double offset = pastShooterAngle.turretRotation * (180.0 / Math.PI) + alphabotJankboticsOffset;
        rotation += offset + 255; //maybe 255
        rotation = rotation % 360;

        double turretAngle = -(rotation / 360.0);

        turret.setTurretPosition(turretAngle);

        double hoodInverted = 85 - (pastShooterAngle.hoodRotation * 180) / Math.PI;
        // double hoodInverted = 40;
        double hoodRelative = hoodInverted / 360;
        turret.setHoodPosition(hoodRelative);

        if (turret.hoodInTolerance(.05) && turret.turretInTolerance(0.05)) {
            return true;
        }
        return false;
    }

    // public boolean aimFerry(){}

    // public double getTurretPosFromJoystick() {
    //     return tIO.getTurretPosition()
    //             + 0.05 * MathUtil.applyDeadband(coDriverController.getLeftX(), Constants.leftYDeadband);
    // }

    // public double getHoodPosFromJoystick() {
    //     return tIO.getHoodPosition()
    //             + 0.05 * -MathUtil.applyDeadband(coDriverController.getLeftY(), Constants.leftXDeadband);
    // }

    public boolean isHubActive() {
        double timer = Constants.timer.get();
        gameData = DriverStation.getGameSpecificMessage();
        if (gameData.length() > 0) {
            switch (gameData.charAt(0)) {
                case 'B':
                    if (Constants.isBlueAlliance) {
                        return (timer <= 10 || (timer >= (35 - prefire) && timer <= 60)
                                || (timer >= (85 - prefire)));
                    } else {
                        return (timer <= 35) || (timer >= (60 - prefire) && timer <= 85)
                                || (timer >= (110 - prefire));
                    }
                case 'R':
                    if (!Constants.isBlueAlliance) {
                        return (timer <= 35) || (timer >= (60 - prefire) && timer <= 85)
                                || (timer >= (110 - prefire));
                    } else {
                        return (timer <= 10 || (timer >= (35 - prefire) && timer <= 60)
                                || (timer >= (85 - prefire)));
                    }

                default:
                    return true;
            }
        } else {
            return true;
        }
    }

}
