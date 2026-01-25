package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.ShooterAngle;
import frc.robot.subsystems.Vision.ShooterAngleCalculator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.Shooter.Feeder.FeederIO;
import frc.robot.subsystems.Shooter.Feeder.FeederIOInputsAutoLogged;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIO;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOInputsAutoLogged;
import frc.robot.subsystems.Shooter.Turret.TurretIO;
import frc.robot.subsystems.Shooter.Turret.TurretIOInputsAutoLogged;

public class Shooter extends SubsystemBase{
    ShooterStates currentShooterState = ShooterStates.HUB;//SAFE;
    private ShooterAngleCalculator AngleCalculator = new ShooterAngleCalculator();
    private static Shooter instance = null;
    private final FeederIOInputsAutoLogged feederInputs = new FeederIOInputsAutoLogged();
    private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
    private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();
    public final FeederIO fIO;
    public final FlywheelIO fwIO;
    public final TurretIO tIO;
    private Drive drive = Drive.GetInstance();
    private Pose2d pose;
    private ShooterAngle shooterAngle;
    private Pose2d hubPose = new Pose2d(10, 10, new Rotation2d());

    public Shooter(FeederIO fIO,FlywheelIO fwIO, TurretIO tIO){
        this.fIO = fIO;
        this.fwIO = fwIO;
        this.tIO = tIO;
        // Shuffleboard.getTab("Drive")
        //     .add("Vx", 1)
        //     .withWidget(BuiltInWidgets.kNumberSlider)
        //     .getEntry();
        // Shuffleboard.getTab("Drive")
        //     .add("Vy", 1)
        //     .withWidget(BuiltInWidgets.kNumberSlider)
        //     .getEntry();
        // Shuffleboard.getTab("Drive")
        //     .add("Prx", 1)
        //     .withWidget(BuiltInWidgets.kNumberSlider)
        //     .getEntry();
        // Shuffleboard.getTab("Drive")
        //     .add("Prx", 1)
        //     .withWidget(BuiltInWidgets.kNumberSlider)
        //     .getEntry();
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
        pose = drive.getPose();
        ApplyStates();
        handleStateTransitions();
        fIO.updateInputs(feederInputs);
        Logger.processInputs("Feeder",feederInputs);
        fwIO.updateInputs(flywheelInputs);
        Logger.processInputs("Flywheels",flywheelInputs);
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
                shooterAngle = ShooterAngleCalculator.getShooterAngleToHub(
                    drive.getChassisSpeeds().vxMetersPerSecond + 1,
                    drive.getChassisSpeeds().vxMetersPerSecond + 5,
                    hubPose.getX() - pose.getY(),
                    hubPose.getY() - pose.getX(),
                    Flywheel.getInstance().getFlyWheelVelocity().in(Units.RadiansPerSecond) * Flywheel.flywheelRadius
                );
                SmartDashboard.putNumber("shooterHoodAngle",shooterAngle.hoodRotation);
                SmartDashboard.putNumber("shooterHoodAngle",shooterAngle.turretRotation);
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

    
 