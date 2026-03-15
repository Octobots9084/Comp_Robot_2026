package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    public ShooterStates currentShooterState = ShooterStates.SAFE;// SAFE; //should be safe but useing hub for testing
    public ShooterStates wantedShooterState = ShooterStates.SAFE;
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
    public boolean turretAlreadyZeroed = false;
    public boolean hoodAlreadyZeroed = false;
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
    public boolean isAimedAtFerry;
    public double shooterCalculatorVelocity;
    public int flywheelDebouncer = 0;
    public double hubBallSpeed = 6.7;
    public double hubFlywheelSpeed = 10;
    public double ferryBallSpeed = 6.7;
    public double ferryFlywheelSpeed = 10;

    public double manuelHood = 90; 
    public double manuelFlywheel = 12; //0 to 1

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
        switch (currentShooterState){
            case SAFE:
                // stops the flywheel
                feeder.setFeederVelocity(FeederStates.OFF);
                flywheel.setFlywheelVelocity(FlywheelStates.SAFE);
                break;
            case MANUAL:
                // joystick controlls turret and hood
                // tIO.setTurretPosition(getTurretPosFromJoystick());
                // tIO.setHoodPosition(getHoodPosFromJoystick());
                break;
            case UNJAM:
                //unjams the shooter
                flywheel.setFlywheelVelocity(0);
                feeder.setFeederVelocity(FeederStates.UNJAM);
                break;
            case SPITTOCONTAINER:
                feeder.setFeederVelocity(FeederStates.SPITTING);
                flywheel.setFlywheelVelocity(FlywheelStates.SPITTOCONTAINER);
                turret.setHoodPosition(85);
                turret.setTurretPosition(-90/360.0);
                break;
            case FERRY:
                //Shoots balls from mid
                // hubBallSpeed = 6.7 +(8.13-6.7)*(getDistanceToHub()/4.18532579377);
                hubBallSpeed = 8;
                isAimedAtFerry = aimFerry(hubBallSpeed);
                // hubFlywheelSpeed = (hubBallSpeed-0.0482494)/0.673537;
                hubFlywheelSpeed = 11;
                // isAimedAtHub = true;
                // turret.setTurretPosition(-90.0/360.0);
                // turret.setHoodPosition(75/360.0);
                
                if(!swerve.isInAllianceZone()){
                    if(driverOverride){
                        // flywheel.setFlywheelVelocity(7.098+1.34*((getDistanceToHub()-1.237)/(5.476-1.237)));
                        // flywheel.setFlywheelVelocity(10+2*((getDistanceToHub()-1.237)/(5.476-1.237)));
                        flywheel.setFlywheelVelocity(hubFlywheelSpeed);
                        // flywheel.setFlywheelVelocity(FlywheelStates.HUB);
                        if(isAimedAtFerry){
                            if(flywheel.FlywheelInTolerance(1)){
                                feeder.setFeederVelocity(FeederStates.SCORING);
                                flywheelDebouncer = 0;
                            }else if (flywheelDebouncer<10){
                                flywheelDebouncer ++;
                                feeder.setFeederVelocity(FeederStates.SCORING);
                            }
                            else{
                                feeder.setFeederVelocity(FeederStates.OFF);
                            }
                        }
                        
                    }else{
                        feeder.setFeederVelocity(FeederStates.OFF);
                        flywheel.setFlywheelVelocity(FlywheelStates.SAFE);
                    }
                }else{
                    wantedShooterState = ShooterStates.BUMP;
                }
                break;
            // case HUB:
            //     // hubBallSpeed = 
            //     // SmartDashboard.getNumber("hubBallSpeed", 8);
            //     hubBallSpeed = 6.7
            //     ;//7.05 +(8.13-6.7)*(getDistanceToHub()/4.18532579377);
            //     isAimedAtHub = isAimedAtHub(hubBallSpeed);
            //     // hubFlywheelSpeed = (hubBallSpeed-0.0482494)/0.673537;
            //     // hubFlywheelSpeed = SmartDashboard.getNumber("hubFlywheelSpeed", 10.5);
            //     hubFlywheelSpeed = 9.5;


            //     // isAimedAtHub = true;
            //     // turret.setTurretPosition(-90.0/360.0);
            //     // turret.setHoodPosition(75/360.0);
                
            //     if(swerve.isInAllianceZone()){
            //         if(driverOverride){
            //             // flywheel.setFlywheelVelocity(7.098+1.34*((getDistanceToHub()-1.237)/(5.476-1.237)));
            //             // flywheel.setFlywheelVelocity(10+2*((getDistanceToHub()-1.237)/(5.476-1.237)));
            //             flywheel.setFlywheelVelocity(hubFlywheelSpeed);
            //             // flywheel.setFlywheelVelocity(FlywheelStates.HUB);
            //             if(isAimedAtHub){
            //                 if(flywheel.FlywheelInTolerance(1)){
            //                     feeder.setFeederVelocity(FeederStates.SCORING);
            //                     flywheelDebouncer = 0;
            //                 }else if (flywheelDebouncer<10){
            //                     flywheelDebouncer ++;
            //                     feeder.setFeederVelocity(FeederStates.SCORING);
            //                 }
            //                 else{
            //                     feeder.setFeederVelocity(FeederStates.OFF);
            //                 }
            //             }
                        
            //         }else{
            //             feeder.setFeederVelocity(FeederStates.OFF);
            //             flywheel.setFlywheelVelocity(FlywheelStates.SAFE);
            //         }
            //     }else{
            //         wantedShooterState = ShooterStates.BUMP;
            //     }
            //     break;
            case HUB:
                Logger.recordOutput("manuel hood position", manuelHood);
                Logger.recordOutput("manuel flywheel position", manuelFlywheel);
                isAimedAtHub(30);
                turret.setHoodPosition(manuelHood/360.0);

                

                    if(driverOverride){
                        flywheel.setFlywheelVelocity(manuelFlywheel);
                        if(flywheel.FlywheelInTolerance(1)){
                                feeder.setFeederVelocity(FeederStates.SCORING);
                                flywheelDebouncer = 0;
                            }else if (flywheelDebouncer<10){
                                flywheelDebouncer ++;
                                feeder.setFeederVelocity(FeederStates.SCORING);
                            }
                            else{
                                feeder.setFeederVelocity(FeederStates.OFF);
                            }

                        
                    }else{
                        feeder.setFeederVelocity(FeederStates.OFF);
                        flywheel.setFlywheelVelocity(FlywheelStates.SAFE);
                    }
                break;
            case AUTOFERRY:
                ferryBallSpeed = 6.7 +(8.13-6.7)*(getDistanceToHub()/4.18532579377);
                isAimedAtHub = isAimedAtHub(hubBallSpeed);
                ferryFlywheelSpeed = (ferryBallSpeed-0.0482494)/0.673537;
                if(swerve.isInAllianceZone()){
                        flywheel.setFlywheelVelocity(ferryFlywheelSpeed);
                        if(isAimedAtHub){
                            if(flywheel.FlywheelInTolerance(1)){
                                feeder.setFeederVelocity(FeederStates.SCORING);
                                flywheelDebouncer = 0;
                            }else if (flywheelDebouncer<10){
                                flywheelDebouncer ++;
                                feeder.setFeederVelocity(FeederStates.SCORING);
                            }
                            else{
                                feeder.setFeederVelocity(FeederStates.OFF);
                            }
                        }
                }else{
                    wantedShooterState = ShooterStates.BUMP;
                }
                break;
            case AUTOHUB:
                //hubBallSpeed = 6.7 +(8.13-6.7)*(getDistanceToHub()/4.18532579377);
                hubBallSpeed = 0.5*(getDistanceToHub())+6;//8;//12;//6.5361+ 0.96897 * (getDistanceToHub()); // first change
                hubFlywheelSpeed = 1.333333*getDistanceToHub()+5.133333;//7;//8.5;//6.3677 + 0.56653 * (getDistanceToHub()); // second change
                Logger.recordOutput("hubBallSpeed",hubBallSpeed);
                Logger.recordOutput("hubFlywheelSpeed",hubFlywheelSpeed);
                isAimedAtHub = isAimedAtHub(hubBallSpeed);
                if(swerve.isInAllianceZone()){
                    flywheel.setFlywheelVelocity(hubFlywheelSpeed);
                    if(isAimedAtHub){
                        if(flywheel.FlywheelInTolerance(1)){
                            feeder.setFeederVelocity(FeederStates.SCORING);
                            flywheelDebouncer = 0;
                            }else if (flywheelDebouncer<10){
                                flywheelDebouncer ++;
                                feeder.setFeederVelocity(FeederStates.SCORING);
                            }
                            else{
                                feeder.setFeederVelocity(FeederStates.OFF);
                            }
                        }
                }else{
                    wantedShooterState = ShooterStates.BUMP;
                }
            case BUMP:
                if (!swerve.isTilted(0, 5)) {
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
            case FIXEDFIRE:
                feeder.setFeederVelocity(FeederStates.SCORING);
                flywheel.setFlywheelVelocity(FlywheelStates.HUB);
                turret.setHoodPosition(80/360.0);
                turret.setTurretPosition(0.25);
                break;
            case ZERO:
                turret.io.zeroHoodMotor();
                if (turret.io.turretZeroed()) {
                    wantedShooterState = ShooterStates.HUB;
                }
                break;
            default:
                break;
        }

    }

    public void handleStateTransitions() {
        if ((wantedShooterState != ShooterStates.HUB && currentShooterState == ShooterStates.HUB)|| (wantedShooterState != ShooterStates.FERRY && currentShooterState == ShooterStates.FERRY)){
            flywheelDebouncer = 0;
        }
        switch (wantedShooterState) {
            case HUB:
                // if we're on our side of the field
                if (swerve.isInAllianceZone()) {// !tilted and in alliance
                    currentShooterState = ShooterStates.HUB;
                }else{
                    currentShooterState = ShooterStates.FERRY;
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
                if (!swerve.isInAllianceZone()) {// !tilted and !in alliance
                    currentShooterState = ShooterStates.FERRY;
                }else{
                    currentShooterState = ShooterStates.HUB;
                }
                break;

            case BUMP:
                // if we're on the bump
                currentShooterState = ShooterStates.BUMP;
                break;

            case SAFE:
                // driver input
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
                break;
            case UNJAM:
                currentShooterState = ShooterStates.UNJAM;
                break;
            default:
                break;
        }
        ;
    }
    
    public double getDistanceToHub(){
        double XToHub;
        double YToHub;
        if(Constants.isBlueAlliance){
            YToHub = getYToTarget(hubPoseBlue.getY());
            XToHub =  getXToTarget(hubPoseBlue.getX());
        } else {
            YToHub = getYToTarget(hubPoseRed.getY());
            XToHub =  getXToTarget(hubPoseRed.getX());
        }
        return Math.sqrt(YToHub*YToHub+XToHub*XToHub);
    }

    public double getYToTarget(double poseY){
        return poseY - (swerve.io.getPose2d().getY()
             + Constants.TurretDistFromCenter
                * Math.sin(((swerve.io.getPose2d().getRotation().getRadians())) + Constants.TurretAngleFromCenter));
    }

    public double getXToTarget(double poseX){
        return poseX - (swerve.io.getPose2d().getX()
            + Constants.TurretDistFromCenter
                * Math.cos(((swerve.io.getPose2d().getRotation().getRadians())) + Constants.TurretAngleFromCenter));
    }


    public double getY(double hubPoseY){
        
        return swerve.io.getPose2d().getY()
             + Constants.TurretDistFromCenter
                * Math.sin(((swerve.io.getPose2d().getRotation().getRadians())) + Constants.TurretAngleFromCenter);
    }

    public double getX(double hubPoseX){
        
        return swerve.io.getPose2d().getX()
            + Constants.TurretDistFromCenter
                * Math.cos(((swerve.io.getPose2d().getRotation().getRadians())) + Constants.TurretAngleFromCenter);
    }

    public boolean isAimedAtHub(double flywheelSpeedSetpoint) {
        double YToHub;
        double XToHub;
        if (Constants.isBlueAlliance) {
            YToHub = getYToTarget(hubPoseBlue.getY());
            XToHub = getXToTarget(hubPoseBlue.getX());
        } else {
            YToHub = getYToTarget(hubPoseRed.getY());
            XToHub = getXToTarget(hubPoseRed.getX());
        }

        Logger.recordOutput("ToHub",new Translation2d(getXToTarget(hubPoseRed.getX()),getYToTarget(hubPoseRed.getY())));
        ChassisSpeeds fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.io.getChassisSpeeds(), swerve.io.getPose2d().getRotation());


        shooterAngle = ShooterAngleCalculator.getShooterAngleToHub(
                    fieldRelative.vxMetersPerSecond,
                    fieldRelative.vyMetersPerSecond,
                    XToHub,
                    YToHub,
                    flywheelSpeedSetpoint
            );

        if (shooterAngle != null) {
            pastShooterAngle = shooterAngle;
        }
        // SmartDashboard.putNumber("shooterHoodAngle", pastShooterAngle.hoodRotation);
        // SmartDashboard.putNumber("shooterAngle", pastShooterAngle.turretRotation);

        double rotation = SwerveSubsystem.getInstance().getRobotPose().getRotation().getRadians();

        rotation = rotation-Math.PI/2;

        double proposedAngle = (((pastShooterAngle.turretRotation - rotation) + Math.PI) % (2*Math.PI) - Math.PI);
        Logger.recordOutput("ProposedAngle", 180*proposedAngle/(Math.PI));

        if (
            (proposedAngle - 2*Math.PI) > Constants.minTurretAngle
            &&
            Math.abs(tIO.getTurretPosition()-(proposedAngle - 2*Math.PI)) < Math.abs(tIO.getTurretPosition()-proposedAngle)
            )
        {
            proposedAngle = proposedAngle - 2*Math.PI;
        }
        else if (
            (proposedAngle + 2*Math.PI) < Constants.maxTurretAngle
            &&
            Math.abs(tIO.getTurretPosition()-(proposedAngle + 2*Math.PI)) < Math.abs(tIO.getTurretPosition()-proposedAngle)
            )
        {
            proposedAngle = proposedAngle + 2*Math.PI;
        }

        turret.setTurretPosition(proposedAngle/(2*Math.PI));
        // turret.setTurretPosition(-0.25);
                
        Logger.recordOutput("CalculatedCorrectedTurretAngle", 180*proposedAngle/(Math.PI));


        /* Angle Compensator??, TODO ask rui why the **** this is commented

            double rotation = -180;
            rotation = rotation % 360;
            rotation = 360 - rotation;

            // Invert gyro direction BEFORE scaling
            double offset = 190;//pastShooterAngle.turretRotation * (180.0 / Math.PI);
            double offset = (pastShooterAngle.turretRotation * (180.0 / Math.PI));// + alphabotJankboticsOffset;
            rotation += offset + 255; //maybe 255
            rotation = rotation % 360;
            double turretAngle = -(rotation / 360.0);

            turret.setTurretPosition(turretAngle);
            double hoodInverted = 85 - (pastShooterAngle.hoodRotation * 180) / Math.PI;
            double hoodInverted = 85-(67);
            double hoodRelative = hoodInverted / 360;
        */

        turret.setHoodPosition(pastShooterAngle.hoodRotation/(2.0*Math.PI));
        // turret.setHoodPosition(75/360.0);

        Logger.recordOutput("CalculatedHoodAngle", pastShooterAngle.hoodRotation/(2*Math.PI));

        return true;
        // return (turret.hoodInTolerance(.05) && turret.turretInTolerance(0.05));
    }



    public boolean aimFerry(double flywheelSpeedSetpoint) {
        double YToHub;
        double XToHub;
        if(!Constants.isBlueAlliance){
            double redFerryDepotDistance = Math.sqrt(Math.pow(redFerryDepot.getX() - swerve.io.getPose2d().getX(),2)+Math.pow((redFerryDepot.getY() - swerve.io.getPose2d().getY()),2));
            double redFerryOutpostDistance = Math.sqrt(Math.pow(redFerryOutpost.getX() - swerve.io.getPose2d().getX(),2)+Math.pow((redFerryOutpost.getY() - swerve.io.getPose2d().getY()),2));
            if(redFerryDepotDistance <= redFerryOutpostDistance){
                XToHub = getXToTarget(redFerryDepot.getX());
                YToHub = getYToTarget(redFerryDepot.getY());
            }else{
                XToHub = getXToTarget(redFerryOutpost.getX());
                YToHub = getYToTarget(redFerryOutpost.getY());
            }
        }else{
            double blueFerryDepotDistance = Math.sqrt(Math.pow(blueFerryDepot.getX() - swerve.io.getPose2d().getX(),2)+Math.pow((blueFerryDepot.getY() - swerve.io.getPose2d().getY()),2));
            double blueFerryOutpostDistance = Math.sqrt(Math.pow(blueFerryOutpost.getX() - swerve.io.getPose2d().getX(),2)+Math.pow((blueFerryOutpost.getY() - swerve.io.getPose2d().getY()),2));
            if(blueFerryDepotDistance <= blueFerryOutpostDistance){
                XToHub = getXToTarget(blueFerryDepot.getX());
                YToHub = getYToTarget(blueFerryDepot.getY());
            }else{
                XToHub = getXToTarget(blueFerryOutpost.getX());
                YToHub = getYToTarget(blueFerryOutpost.getY());
            }
        }

        Logger.recordOutput("ToHub",new Translation2d(getXToTarget(hubPoseRed.getX()),getYToTarget(hubPoseRed.getY())));

        ChassisSpeeds fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.io.getChassisSpeeds(), swerve.io.getPose2d().getRotation());


        shooterAngle = ShooterAngleCalculator.getShooterAngleToFerry(
                    fieldRelative.vxMetersPerSecond,
                    fieldRelative.vyMetersPerSecond,
                    XToHub,
                    YToHub,
                    flywheelSpeedSetpoint
            );

        if (shooterAngle != null) {
            pastShooterAngle = shooterAngle;
        }
        // SmartDashboard.putNumber("shooterHoodAngle", pastShooterAngle.hoodRotation);
        // SmartDashboard.putNumber("shooterAngle", pastShooterAngle.turretRotation);

        double rotation = SwerveSubsystem.getInstance().getRobotPose().getRotation().getRadians();

        rotation = rotation-Math.PI/2;

        double proposedAngle = (((pastShooterAngle.turretRotation - rotation) + Math.PI) % (2*Math.PI) - Math.PI);
        Logger.recordOutput("ProposedAngle", 180*proposedAngle/(Math.PI));

        if (
            (proposedAngle - 2*Math.PI) > Constants.minTurretAngle
            &&
            Math.abs(tIO.getTurretPosition()-(proposedAngle - 2*Math.PI)) < Math.abs(tIO.getTurretPosition()-proposedAngle)
            )
        {
            proposedAngle = proposedAngle - 2*Math.PI;
        }
        else if (
            (proposedAngle + 2*Math.PI) < Constants.maxTurretAngle
            &&
            Math.abs(tIO.getTurretPosition()-(proposedAngle + 2*Math.PI)) < Math.abs(tIO.getTurretPosition()-proposedAngle)
            )
        {
            proposedAngle = proposedAngle + 2*Math.PI;
        }

        turret.setTurretPosition(proposedAngle/(2*Math.PI));
        // turret.setTurretPosition(-0.25);
        
        
        Logger.recordOutput("CalculatedCorrectedTurretAngle", 180*proposedAngle/(Math.PI));

         /* Angle Compensator?? why is it here too!!??, TODO ask rui why the **** this is commented

            double rotation = -180;
            rotation = rotation % 360;
            rotation = 360 - rotation;

            // Invert gyro direction BEFORE scaling
            double offset = 190;//pastShooterAngle.turretRotation * (180.0 / Math.PI);
            double offset = (pastShooterAngle.turretRotation * (180.0 / Math.PI));// + alphabotJankboticsOffset;
            rotation += offset + 255; //maybe 255
            rotation = rotation % 360;
            double turretAngle = -(rotation / 360.0);

            turret.setTurretPosition(turretAngle);
            double hoodInverted = 85 - (pastShooterAngle.hoodRotation * 180) / Math.PI;
            double hoodInverted = 85-(67);
            double hoodRelative = hoodInverted / 360;
        */

        turret.setHoodPosition(pastShooterAngle.hoodRotation/(2.0*Math.PI));
        // turret.setHoodPosition(75/360.0);

        Logger.recordOutput("CalculatedHoodAngle", pastShooterAngle.hoodRotation/(2*Math.PI));

        return true;
        // return (turret.hoodInTolerance(.05) && turret.turretInTolerance(0.05));
    }

    // public double getTurretPosFromJoystick() {
    //     return tIO.getTurretPosition()
    //              + 0.05 * MathUtil.applyDeadband(coDriverController.getLeftX(), Constants.leftYDeadband);
    // }

    // public double getHoodPosFromJoystick() {
    //      return tIO.getHoodPosition()
    //              + 0.05 * -MathUtil.applyDeadband(coDriverController.getLeftY(), Constants.leftXDeadband);
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